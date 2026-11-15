// license:GPLv3+

#include "B2SProDynamicLight.h"
#include "B2SProData.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <execution>
#include <numeric>

namespace B2S {

// VB CInt / Math.Round round half to even; nearbyint does the same in the default rounding mode
static inline int RoundEven(double v) { return static_cast<int>(std::nearbyint(v)); }
static inline uint8_t ClampByte(int v) { return static_cast<uint8_t>(std::clamp(v, 0, 255)); }
static inline double SmoothStep(double value)
{
   const double t = std::clamp(value, 0.0, 1.0);
   return t * t * (3.0 - 2.0 * t);
}
static inline double Lerp(double a, double b, double amount) { return a + (b - a) * std::clamp(amount, 0.0, 1.0); }

static void ParallelRows(int count, const std::function<void(int)>& body)
{
   vector<int> rows(static_cast<size_t>(std::max(0, count)));
   std::iota(rows.begin(), rows.end(), 0);
   std::for_each(std::execution::par, rows.begin(), rows.end(), [&body](int y) { body(y); });
}

// BlurDoubleMapWithWorkspace: box blur, horizontal then vertical, radius in pixels
static void BlurDoubleMap(const vector<double>& values, int width, int height, int radius, vector<double>& horizontal, vector<double>& output)
{
   horizontal.assign(values.size(), 0.0);
   output.assign(values.size(), 0.0);
   ParallelRows(height,
      [&](int y)
      {
         const int row = y * width;
         double sum = 0.0;
         int left = 0;
         int right = std::min(width - 1, radius);
         for (int x = left; x <= right; x++)
            sum += values[row + x];
         for (int x = 0; x < width; x++)
         {
            horizontal[row + x] = sum / (right - left + 1);
            const int removeX = x - radius;
            const int addX = x + radius + 1;
            if (removeX >= 0)
            {
               sum -= values[row + removeX];
               left++;
            }
            if (addX < width)
            {
               sum += values[row + addX];
               right++;
            }
         }
      });
   ParallelRows(width,
      [&](int x)
      {
         double sum = 0.0;
         int top = 0;
         int bottom = std::min(height - 1, radius);
         for (int y = top; y <= bottom; y++)
            sum += horizontal[y * width + x];
         for (int y = 0; y < height; y++)
         {
            output[y * width + x] = sum / (bottom - top + 1);
            const int removeY = y - radius;
            const int addY = y + radius + 1;
            if (removeY >= 0)
            {
               sum -= horizontal[removeY * width + x];
               top++;
            }
            if (addY < height)
            {
               sum += horizontal[addY * width + x];
               bottom++;
            }
         }
      });
}

void ProRenderDynamicLighting(const uint8_t* source, const uint8_t* lightField, int width, int height, int intensity, int lightR, int lightG, int lightB, int flasherStyle,
   int saturation, int highlightProtection, int darkAreaLift, int transmissionContrast, int lightTemperature, vector<uint8_t>& output)
{
   const size_t pixelCount = static_cast<size_t>(width) * static_cast<size_t>(height);
   output.assign(pixelCount * 4, 0);
   if (pixelCount == 0)
      return;

   // BuildTransmissionMapWithWorkspace
   vector<double> luminance(pixelCount);
   ParallelRows(height,
      [&](int y)
      {
         for (int x = 0; x < width; x++)
         {
            const uint8_t* p = source + (static_cast<size_t>(y) * width + x) * 4;
            luminance[static_cast<size_t>(y) * width + x] = std::clamp((p[0] * 0.299 + p[1] * 0.587 + p[2] * 0.114) / 255.0, 0.0, 1.0);
         }
      });
   vector<double> horizontal, blurred;
   BlurDoubleMap(luminance, width, height, 4, horizontal, blurred);
   const double contrast = std::clamp(transmissionContrast / 100.0, 0.0, 3.0);
   const double detailStrength = 0.45 + contrast * 1.15;
   const double midpointStrength = 0.72 + contrast * 0.28;

   // RenderPixelsWithWorkspace
   const double normalizedIntensity = std::clamp(intensity / 100.0, 0.0, 16.0);
   const double exposureStrength = normalizedIntensity * 1.18 + std::pow(std::max(0.0, normalizedIntensity - 3.0), 1.28) * 0.58;
   const double protect = std::clamp(highlightProtection / 100.0, 0.0, 1.0);
   const double shadowLift = std::clamp(darkAreaLift / 100.0, 0.0, 1.0);
   const double sat = std::clamp(saturation / 100.0, 0.0, 2.0);
   ParallelRows(height,
      [&](int y)
      {
         for (int x = 0; x < width; x++)
         {
            const size_t index = static_cast<size_t>(y) * width + x;
            const double field = lightField[index] / 255.0;
            if (field <= 0.0)
               continue;
            const uint8_t* sp = source + index * 4;
            uint8_t* dp = output.data() + index * 4;
            const double r = sp[0];
            const double g = sp[1];
            const double b = sp[2];
            const double sourceAlpha = sp[3] / 255.0;
            const double transmitted = std::pow(std::max(0.0, luminance[index]), 0.82) * midpointStrength + (luminance[index] - blurred[index]) * detailStrength;
            const double surfaceDetail = std::clamp(transmitted, 0.025, 1.0);
            const double lum = (r * 0.299 + g * 0.587 + b * 0.114) / 255.0;
            const double blackProtection = SmoothStep((std::max(r, std::max(g, b)) - 18.0) / 30.0);
            if (blackProtection <= 0.0)
               continue;
            const double response = 0.72 + shadowLift * (1.0 - lum) * 0.28;
            const double detailResponse = 0.8 + surfaceDetail * 0.4;
            double localEnergy = exposureStrength * field * response * detailResponse;
            localEnergy *= 1.0 - protect * lum * 0.8;
            const double gain = 1.0 + localEnergy * 0.38;
            double rr = r * (1.0 + (gain - 1.0) * lightR / 255.0);
            double gg = g * (1.0 + (gain - 1.0) * lightG / 255.0);
            double bb = b * (1.0 + (gain - 1.0) * lightB / 255.0);
            const double burstMix = SmoothStep((normalizedIntensity - 8.0) / 8.0) * field * SmoothStep((lum - 0.55) / 0.45);
            rr += (255.0 - rr) * burstMix;
            gg += (255.0 - gg) * burstMix;
            bb += (255.0 - bb) * burstMix;
            const double litLum = rr * 0.299 + gg * 0.587 + bb * 0.114;
            rr = litLum + (rr - litLum) * sat;
            gg = litLum + (gg - litLum) * sat;
            bb = litLum + (bb - litLum) * sat;
            if (flasherStyle == 2)
            {
               rr = lightR;
               gg = lightG;
               bb = lightB;
            }
            dp[0] = ClampByte(RoundEven(rr));
            dp[1] = ClampByte(RoundEven(gg));
            dp[2] = ClampByte(RoundEven(bb));
            dp[3] = ClampByte(RoundEven(field * sourceAlpha * blackProtection * 255.0));
         }
      });

   // ApplyTemperatureWithWorkspace
   const int kelvin = std::clamp(lightTemperature, 2000, 6500);
   if (kelvin == 4000)
      return;
   double rf, gf, bf;
   if (kelvin <= 2700)
   {
      const double t = (kelvin - 2000.0) / 700.0;
      rf = Lerp(1.22, 1.14, t);
      gf = Lerp(0.92, 0.96, t);
      bf = Lerp(0.58, 0.72, t);
   }
   else if (kelvin <= 3000)
   {
      const double t = (kelvin - 2700.0) / 300.0;
      rf = Lerp(1.14, 1.1, t);
      gf = Lerp(0.96, 0.98, t);
      bf = Lerp(0.72, 0.8, t);
   }
   else if (kelvin <= 4000)
   {
      const double t = (kelvin - 3000.0) / 1000.0;
      rf = Lerp(1.1, 1.0, t);
      gf = Lerp(0.98, 1.0, t);
      bf = Lerp(0.8, 1.0, t);
   }
   else if (kelvin <= 5000)
   {
      const double t = (kelvin - 4000.0) / 1000.0;
      rf = Lerp(1.0, 0.96, t);
      gf = Lerp(1.0, 1.01, t);
      bf = Lerp(1.0, 1.1, t);
   }
   else
   {
      const double t = (kelvin - 5000.0) / 1500.0;
      rf = Lerp(0.96, 0.9, t);
      gf = Lerp(1.01, 1.0, t);
      bf = Lerp(1.1, 1.22, t);
   }
   ParallelRows(height,
      [&](int y)
      {
         for (int x = 0; x < width; x++)
         {
            uint8_t* p = output.data() + (static_cast<size_t>(y) * width + x) * 4;
            if (p[3] == 0)
               continue;
            const double orr = p[0], og = p[1], ob = p[2];
            const double oldLuma = orr * 0.2126 + og * 0.7152 + ob * 0.0722;
            double nr = orr * rf, ng = og * gf, nb = ob * bf;
            const double newLuma = nr * 0.2126 + ng * 0.7152 + nb * 0.0722;
            if (newLuma > 0.001)
            {
               const double scale = std::clamp(oldLuma / newLuma, 0.82, 1.18);
               nr *= scale;
               ng *= scale;
               nb *= scale;
            }
            const double strength = 0.55 + 0.3 * (oldLuma / 255.0);
            p[0] = ClampByte(RoundEven(orr + (nr - orr) * strength));
            p[1] = ClampByte(RoundEven(og + (ng - og) * strength));
            p[2] = ClampByte(RoundEven(ob + (nb - ob) * strength));
         }
      });
}

B2SProDynamicLighting::~B2SProDynamicLighting()
{
   for (auto& [key, lit] : m_lit)
      if (lit.tex != nullptr)
         DeleteTexture(lit.tex);
}

bool B2SProDynamicLighting::ReadTexture(VPXTexture tex, Rgba& out)
{
   const VPXTextureInfo* info = tex ? GetTextureInfo(tex) : nullptr;
   if (info == nullptr || info->data == nullptr || info->width == 0 || info->height == 0)
      return false;
   out.w = static_cast<int>(info->width);
   out.h = static_cast<int>(info->height);
   const size_t count = static_cast<size_t>(out.w) * static_cast<size_t>(out.h);
   out.px.resize(count * 4);
   const uint8_t* src = static_cast<const uint8_t*>(info->data);
   switch (info->format)
   {
   case VPXTEXFMT_sRGBA8: memcpy(out.px.data(), src, count * 4); return true;
   case VPXTEXFMT_sRGB8:
      for (size_t i = 0; i < count; i++)
      {
         out.px[i * 4 + 0] = src[i * 3 + 0];
         out.px[i * 4 + 1] = src[i * 3 + 1];
         out.px[i * 4 + 2] = src[i * 3 + 2];
         out.px[i * 4 + 3] = 255;
      }
      return true;
   default: return false;
   }
}

// Bilinear sample of a straight alpha RGBA image at pixel position (u, v), transparent outside
static inline void SampleBilinear(const B2SProDynamicLighting::Rgba& img, double u, double v, uint8_t* out)
{
   const double fx = u - 0.5, fy = v - 0.5;
   const int x0 = static_cast<int>(std::floor(fx)), y0 = static_cast<int>(std::floor(fy));
   const double tx = fx - x0, ty = fy - y0;
   double acc[4] = { 0, 0, 0, 0 };
   double weightAlpha = 0.0;
   for (int k = 0; k < 4; k++)
   {
      const int x = x0 + (k & 1), y = y0 + (k >> 1);
      const double w = ((k & 1) ? tx : 1.0 - tx) * ((k >> 1) ? ty : 1.0 - ty);
      if (x < 0 || y < 0 || x >= img.w || y >= img.h || w <= 0.0)
         continue;
      const uint8_t* p = img.px.data() + (static_cast<size_t>(y) * img.w + x) * 4;
      const double a = p[3] * w;
      acc[0] += p[0] * a;
      acc[1] += p[1] * a;
      acc[2] += p[2] * a;
      weightAlpha += a;
   }
   if (weightAlpha <= 0.0)
   {
      out[0] = out[1] = out[2] = out[3] = 0;
      return;
   }
   out[0] = ClampByte(RoundEven(acc[0] / weightAlpha));
   out[1] = ClampByte(RoundEven(acc[1] / weightAlpha));
   out[2] = ClampByte(RoundEven(acc[2] / weightAlpha));
   out[3] = ClampByte(RoundEven(weightAlpha));
}

// Current rectangle of a picture (a moving B2S Pro picture is at its motion rectangle)
static void CurrentRect(const B2SBulb& bulb, double& x, double& y, double& w, double& h)
{
   if (bulb.m_pro && bulb.m_pro->rt.motion.active)
   {
      const ProMotion& m = bulb.m_pro->rt.motion;
      x = m.x;
      y = m.y;
      w = m.w;
      h = m.h;
      return;
   }
   x = bulb.m_locationX;
   y = bulb.m_locationY;
   w = bulb.m_width;
   h = bulb.m_height;
}

void B2SProDynamicLighting::SnippetFrameCrop(const B2SBulb& snippet, const Rgba& image, int ox, int oy, int ow, int oh, vector<uint8_t>& out)
{
   // The snippet image is stretched over its rectangle and turned around its pivot in authored
   // coordinates (NativeRotationDestinationPoints / CreateCurrentSnippetFrame)
   out.assign(static_cast<size_t>(ow) * static_cast<size_t>(oh) * 4, 0);
   double sx, sy, w, h;
   CurrentRect(snippet, sx, sy, w, h);
   if (w <= 0.0 || h <= 0.0 || image.w <= 0 || image.h <= 0)
      return;
   const bool nativePivot = snippet.m_pro && snippet.m_pro->rt.driven && snippet.m_pro->rt.nativeRotation;
   const double px = nativePivot ? snippet.m_pro->pivotX : 0.5;
   const double py = nativePivot ? snippet.m_pro->pivotY : 0.5;
   const double angle = static_cast<double>(snippet.m_drawRotation) * 3.14159265358979323846 / 180.0;
   const double c = std::cos(angle), s = std::sin(angle);
   const double hingeX = sx + w * px;
   const double hingeY = sy + h * py;
   const double su = image.w / w, sv = image.h / h;
   ParallelRows(oh,
      [&](int j)
      {
         for (int i = 0; i < ow; i++)
         {
            const double dx = ox + i + 0.5 - hingeX;
            const double dy = oy + j + 0.5 - hingeY;
            const double lx = dx * c + dy * s;
            const double ly = -dx * s + dy * c;
            SampleBilinear(image, (lx + w * px) * su, (ly + h * py) * sv, out.data() + (static_cast<size_t>(j) * ow + i) * 4);
         }
      });
}

const B2SProDynamicLighting::Field& B2SProDynamicLighting::GetField(const B2SBulb& light)
{
   Field& field = m_fields[&light];
   if (!field.loaded)
   {
      field.loaded = true;
      if (light.m_pro && !light.m_pro->dynamicFrontLightFieldPng.empty())
      {
         vector<uint8_t> png = light.m_pro->dynamicFrontLightFieldPng;
         if (VPXTexture tex = CreateTexture(png.data(), static_cast<int>(png.size())))
         {
            Rgba rgba;
            if (ReadTexture(tex, rgba))
            {
               field.w = rgba.w;
               field.h = rgba.h;
               field.alpha.resize(static_cast<size_t>(rgba.w) * rgba.h);
               for (size_t i = 0; i < field.alpha.size(); i++)
                  field.alpha[i] = rgba.px[i * 4 + 3];
            }
            DeleteTexture(tex);
         }
      }
   }
   return field;
}

const B2SProDynamicLighting::Rgba* B2SProDynamicLighting::GetImage(const B2SBulb& snippet)
{
   auto it = m_images.find(&snippet);
   if (it == m_images.end())
   {
      Rgba rgba;
      ReadTexture(snippet.m_image, rgba);
      it = m_images.emplace(&snippet, std::move(rgba)).first;
   }
   return it->second.w > 0 ? &it->second : nullptr;
}

void B2SProDynamicLighting::DrawLight(VPXRenderContext2D* ctx, const B2SBulb& light, const vector<std::unique_ptr<B2SBulb>>& candidates, B2SDualMode dualMode, float alpha)
{
   const B2SProBulbData& lp = *light.m_pro;
   const int lx = light.m_locationX, ly = light.m_locationY, lw = light.m_width, lh = light.m_height;
   if (lw <= 0 || lh <= 0)
      return;
   const Field& field = GetField(light);
   if (field.w <= 0 || field.h <= 0)
      return;
   for (const auto& candidate : candidates)
   {
      const B2SBulb& snippet = *candidate;
      if (&snippet == &light || !snippet.m_isImageSnippit || snippet.m_brightness <= 0.f || snippet.m_image == nullptr || snippet.m_zOrder >= light.m_zOrder)
         continue;
      const int snippetId = snippet.m_pro ? snippet.m_pro->id : -1;
      if (lp.dynamicFrontTargetSelectionDefined)
      {
         if (std::find(lp.dynamicFrontTargetSnippetIds.begin(), lp.dynamicFrontTargetSnippetIds.end(), snippetId) == lp.dynamicFrontTargetSnippetIds.end())
            continue;
      }
      else if (lp.dynamicFrontTargetSnippetId > 0 && snippetId != lp.dynamicFrontTargetSnippetId)
         continue;
      if (dualMode != B2SDualMode::Both && snippet.m_dualMode != B2SDualMode::Both && snippet.m_dualMode != dualMode)
         continue;

      // Rectangle.Round of the snippet's current rectangle
      double rsx, rsy, rsw, rsh;
      CurrentRect(snippet, rsx, rsy, rsw, rsh);
      const int snX = static_cast<int>(std::lround(rsx)), snY = static_cast<int>(std::lround(rsy));
      const int snW = static_cast<int>(std::lround(rsw)), snH = static_cast<int>(std::lround(rsh));
      const int ox = std::max(lx, snX), oy = std::max(ly, snY);
      const int ow = std::min(lx + lw, snX + snW) - ox;
      const int oh = std::min(ly + lh, snY + snH) - oy;
      if (ow <= 0 || oh <= 0)
         continue;
      const Rgba* image = GetImage(snippet);
      if (image == nullptr)
         continue;

      const bool driven = snippet.m_pro && snippet.m_pro->rt.driven && snippet.m_pro->rt.nativeRotation;
      Lit& lit = m_lit[{ &light, &snippet }];
      const float pivotX = driven ? snippet.m_pro->pivotX : 0.5f, pivotY = driven ? snippet.m_pro->pivotY : 0.5f;
      const bool same = lit.valid && lit.image == snippet.m_image && lit.sx == snX && lit.sy == snY && lit.sw == snW && lit.sh == snH
         && lit.angle == snippet.m_drawRotation && lit.pivotX == pivotX && lit.pivotY == pivotY && lit.ox == ox && lit.oy == oy && lit.ow == ow && lit.oh == oh;
      if (!same)
      {
         vector<uint8_t> crop;
         SnippetFrameCrop(snippet, *image, ox, oy, ow, oh, crop);
         // BuildDynamicFieldCrop: the overlap part of the light field, resampled to the overlap size
         vector<uint8_t> fieldCrop(static_cast<size_t>(ow) * oh);
         const double fsx = static_cast<double>(field.w) / lw, fsy = static_cast<double>(field.h) / lh;
         for (int j = 0; j < oh; j++)
            for (int i = 0; i < ow; i++)
            {
               const double u = (ox - lx + i + 0.5) * fsx - 0.5, v = (oy - ly + j + 0.5) * fsy - 0.5;
               const int x0 = static_cast<int>(std::floor(u)), y0 = static_cast<int>(std::floor(v));
               const double tx = u - x0, ty = v - y0;
               double acc = 0.0;
               for (int k = 0; k < 4; k++)
               {
                  const int x = std::clamp(x0 + (k & 1), 0, field.w - 1), y = std::clamp(y0 + (k >> 1), 0, field.h - 1);
                  const double wgt = ((k & 1) ? tx : 1.0 - tx) * ((k >> 1) ? ty : 1.0 - ty);
                  acc += field.alpha[static_cast<size_t>(y) * field.w + x] * wgt;
               }
               fieldCrop[static_cast<size_t>(j) * ow + i] = ClampByte(RoundEven(acc));
            }
         vector<uint8_t> lighted;
         ProRenderDynamicLighting(crop.data(), fieldCrop.data(), ow, oh, lp.dynamicLightExposure, lp.dynamicLightColorR, lp.dynamicLightColorG, lp.dynamicLightColorB, lp.dynamicFlasherStyle,
            lp.dynamicSaturation, lp.dynamicHighlightProtection, lp.dynamicDarkAreaLift, lp.dynamicTransmissionContrast, lp.dynamicLightTemperature, lighted);
         UpdateTexture(&lit.tex, ow, oh, VPXTEXFMT_sRGBA8, lighted.data());
         lit.image = snippet.m_image;
         lit.sx = snX;
         lit.sy = snY;
         lit.sw = snW;
         lit.sh = snH;
         lit.angle = snippet.m_drawRotation;
         lit.pivotX = pivotX;
         lit.pivotY = pivotY;
         lit.ox = ox;
         lit.oy = oy;
         lit.ow = ow;
         lit.oh = oh;
         lit.valid = true;
      }
      if (lit.tex != nullptr)
         ctx->DrawImage(ctx, lit.tex, 1.f, 1.f, 1.f, alpha, 0.f, 0.f, static_cast<float>(ow), static_cast<float>(oh), 0.f, 0.f, 0.f, static_cast<float>(ox), static_cast<float>(oy),
            static_cast<float>(ow), static_cast<float>(oh));
   }
}

}
