// license:GPLv3+

#include "B2SProScore.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace B2S {

static inline int RoundEven(double v) { return static_cast<int>(std::nearbyint(v)); }
static inline uint8_t ClampByteD(double v) { return static_cast<uint8_t>(std::clamp(RoundEven(v), 0, 255)); }
static inline double Lerp(double a, double b, double amount) { return a + (b - a) * std::clamp(amount, 0.0, 1.0); }

bool ProReadTexture(VPXTexture tex, ProImage& out)
{
   const VPXTextureInfo* info = tex ? GetTextureInfo(tex) : nullptr;
   if (info == nullptr || info->data == nullptr || info->width == 0 || info->height == 0)
      return false;
   out.Allocate(static_cast<int>(info->width), static_cast<int>(info->height));
   const size_t count = static_cast<size_t>(out.w) * static_cast<size_t>(out.h);
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
   default: out = ProImage(); return false;
   }
}

// GDI+ SourceOver on a non premultiplied ARGB surface
static inline void BlendOver(uint8_t* d, double r, double g, double b, double a)
{
   if (a <= 0.0)
      return;
   const double da = d[3] / 255.0;
   const double sa = a / 255.0;
   const double oa = sa + da * (1.0 - sa);
   if (oa <= 0.0)
   {
      d[0] = d[1] = d[2] = d[3] = 0;
      return;
   }
   d[0] = ClampByteD((r * sa + d[0] * da * (1.0 - sa)) / oa);
   d[1] = ClampByteD((g * sa + d[1] * da * (1.0 - sa)) / oa);
   d[2] = ClampByteD((b * sa + d[2] * da * (1.0 - sa)) / oa);
   d[3] = ClampByteD(oa * 255.0);
}

// Area aware bilinear sample (alpha weighted) of src around (u, v) in source pixels. Outside samples
// are clamped to the edge (clampEdges) or transparent.
static void Sample(const ProImage& src, double u, double v, double footprintU, double footprintV, bool clampEdges, double out[4])
{
   const int nu = std::clamp(static_cast<int>(std::ceil(footprintU)), 1, 8);
   const int nv = std::clamp(static_cast<int>(std::ceil(footprintV)), 1, 8);
   double acc[3] = { 0, 0, 0 };
   double alphaSum = 0.0;
   const double total = static_cast<double>(nu * nv);
   for (int sj = 0; sj < nv; sj++)
      for (int si = 0; si < nu; si++)
      {
         const double su = u + ((si + 0.5) / nu - 0.5) * (nu > 1 ? footprintU : 0.0) - 0.5;
         const double sv = v + ((sj + 0.5) / nv - 0.5) * (nv > 1 ? footprintV : 0.0) - 0.5;
         const int x0 = static_cast<int>(std::floor(su)), y0 = static_cast<int>(std::floor(sv));
         const double tx = su - x0, ty = sv - y0;
         for (int k = 0; k < 4; k++)
         {
            int x = x0 + (k & 1), y = y0 + (k >> 1);
            const double w = ((k & 1) ? tx : 1.0 - tx) * ((k >> 1) ? ty : 1.0 - ty) / total;
            if (w <= 0.0)
               continue;
            if (clampEdges)
            {
               x = std::clamp(x, 0, src.w - 1);
               y = std::clamp(y, 0, src.h - 1);
            }
            else if (x < 0 || y < 0 || x >= src.w || y >= src.h)
               continue;
            const uint8_t* p = src.px.data() + (static_cast<size_t>(y) * src.w + x) * 4;
            const double a = p[3] * w;
            acc[0] += p[0] * a;
            acc[1] += p[1] * a;
            acc[2] += p[2] * a;
            alphaSum += a;
         }
      }
   if (alphaSum <= 0.0)
   {
      out[0] = out[1] = out[2] = out[3] = 0.0;
      return;
   }
   out[0] = acc[0] / alphaSum;
   out[1] = acc[1] / alphaSum;
   out[2] = acc[2] / alphaSum;
   out[3] = alphaSum;
}

void ProDrawScaled(ProImage& dst, const ProImage& src, double sx, double sy, double sw, double sh, double rx, double ry, double rw, double rh, bool copy)
{
   if (src.Empty() || dst.Empty() || rw <= 0.0 || rh <= 0.0 || sw <= 0.0 || sh <= 0.0)
      return;
   const int x0 = std::max(0, static_cast<int>(std::floor(rx))), x1 = std::min(dst.w, static_cast<int>(std::ceil(rx + rw)));
   const int y0 = std::max(0, static_cast<int>(std::floor(ry))), y1 = std::min(dst.h, static_cast<int>(std::ceil(ry + rh)));
   const double fu = sw / rw, fv = sh / rh;
   for (int y = y0; y < y1; y++)
   {
      const double cy = y + 0.5;
      if (cy < ry || cy >= ry + rh)
         continue;
      for (int x = x0; x < x1; x++)
      {
         const double cx = x + 0.5;
         if (cx < rx || cx >= rx + rw)
            continue;
         double s[4];
         Sample(src, sx + (cx - rx) * fu, sy + (cy - ry) * fv, fu, fv, true, s);
         uint8_t* d = dst.px.data() + (static_cast<size_t>(y) * dst.w + x) * 4;
         if (copy)
         {
            d[0] = ClampByteD(s[0]);
            d[1] = ClampByteD(s[1]);
            d[2] = ClampByteD(s[2]);
            d[3] = ClampByteD(s[3]);
         }
         else
            BlendOver(d, s[0], s[1], s[2], s[3]);
      }
   }
}

void ProDrawScaled(ProImage& dst, const ProImage& src, double rx, double ry, double rw, double rh, bool copy)
{
   ProDrawScaled(dst, src, 0.0, 0.0, src.w, src.h, rx, ry, rw, rh, copy);
}

// ReelImageSizing.FittedContentSize / Fit
ProImage ProReelFit(const ProImage& source, int targetW, int targetH)
{
   ProImage result;
   if (source.Empty() || targetW <= 0 || targetH <= 0)
      return result;
   result.Allocate(targetW, targetH);
   const double scale = std::min(static_cast<double>(targetW) / source.w, static_cast<double>(targetH) / source.h);
   const int cw = std::max(1, RoundEven(source.w * scale));
   const int ch = std::max(1, RoundEven(source.h * scale));
   // The background is the source's top-left pixel (Bitmap.GetPixel(0, 0))
   for (size_t i = 0; i < static_cast<size_t>(targetW) * targetH; i++)
      memcpy(&result.px[i * 4], source.px.data(), 4);
   ProDrawScaled(result, source, (targetW - cw) / 2.0, (targetH - ch) / 2.0, cw, ch, false);
   return result;
}

ProImage ProReelStretchToWidth(const ProImage& source, int targetW, int targetH, int contentWidth)
{
   ProImage result;
   if (source.Empty() || targetW <= 0 || targetH <= 0)
      return result;
   result.Allocate(targetW, targetH);
   const int width = std::max(1, std::min(targetW, contentWidth));
   for (size_t i = 0; i < static_cast<size_t>(targetW) * targetH; i++)
      memcpy(&result.px[i * 4], source.px.data(), 4);
   ProDrawScaled(result, source, (targetW - width) / 2.0, 0.0, width, targetH, false);
   return result;
}

static void ApplyTemperatureGrade(ProImage& img, int kelvin)
{
   kelvin = std::clamp(kelvin, 2000, 6500);
   if (kelvin == 4000)
      return;
   double rf, gf, bf;
   if (kelvin <= 2700)
   {
      const double a = (kelvin - 2000.0) / 700.0;
      rf = Lerp(1.22, 1.14, a);
      gf = Lerp(0.92, 0.96, a);
      bf = Lerp(0.58, 0.72, a);
   }
   else if (kelvin <= 3000)
   {
      const double a = (kelvin - 2700.0) / 300.0;
      rf = Lerp(1.14, 1.1, a);
      gf = Lerp(0.96, 0.98, a);
      bf = Lerp(0.72, 0.8, a);
   }
   else if (kelvin <= 4000)
   {
      const double a = (kelvin - 3000.0) / 1000.0;
      rf = Lerp(1.1, 1.0, a);
      gf = Lerp(0.98, 1.0, a);
      bf = Lerp(0.8, 1.0, a);
   }
   else if (kelvin <= 5000)
   {
      const double a = (kelvin - 4000.0) / 1000.0;
      rf = Lerp(1.0, 0.96, a);
      gf = Lerp(1.0, 1.01, a);
      bf = Lerp(1.0, 1.1, a);
   }
   else
   {
      const double a = (kelvin - 5000.0) / 1500.0;
      rf = Lerp(0.96, 0.9, a);
      gf = Lerp(1.01, 1.0, a);
      bf = Lerp(1.1, 1.22, a);
   }
   for (size_t i = 0; i < static_cast<size_t>(img.w) * img.h; i++)
   {
      uint8_t* p = &img.px[i * 4];
      if (p[3] == 0)
         continue;
      const double ored = p[0], ogreen = p[1], oblue = p[2];
      const double oldLuma = ored * 0.2126 + ogreen * 0.7152 + oblue * 0.0722;
      double gr = ored * rf, gg = ogreen * gf, gb = oblue * bf;
      const double newLuma = gr * 0.2126 + gg * 0.7152 + gb * 0.0722;
      if (newLuma > 0.001)
      {
         const double s = std::clamp(oldLuma / newLuma, 0.82, 1.18);
         gr *= s;
         gg *= s;
         gb *= s;
      }
      const double strength = 0.55 + 0.3 * (oldLuma / 255.0);
      p[0] = ClampByteD(ored + (gr - ored) * strength);
      p[1] = ClampByteD(ogreen + (gg - ogreen) * strength);
      p[2] = ClampByteD(oblue + (gb - oblue) * strength);
   }
}

// Reel3DEffect.RenderReel
ProImage ProReel3DRender(const ProImage& source, int targetW, int targetH, int brightness, int temperature, int depth)
{
   ProImage input;
   if (source.Empty() || targetW <= 0 || targetH <= 0)
      return input;
   brightness = std::clamp(brightness, 0, 400);
   temperature = std::clamp(temperature, 2000, 6500);
   depth = std::clamp(depth, 0, 200);
   input.Allocate(targetW, targetH);
   ProDrawScaled(input, source, 0.0, 0.0, targetW, targetH, true);

   ProImage result;
   result.Allocate(targetW, targetH);
   const double depthAmount = depth / 100.0;
   const double lightAmount = brightness / 100.0;
   for (int y = 0; y < targetH; y++)
   {
      const double normalY = targetH <= 1 ? 0.0 : 2.0 * y / (targetH - 1.0) - 1.0;
      const double roundness = std::sqrt(std::max(0.0, 1.0 - normalY * normalY));
      const double drumShade = std::max(0.18, 1.0 - depthAmount * 0.5 * (1.0 - roundness));
      for (int x = 0; x < targetW; x++)
      {
         const uint8_t* ip = &input.px[(static_cast<size_t>(y) * targetW + x) * 4];
         uint8_t* op = &result.px[(static_cast<size_t>(y) * targetW + x) * 4];
         const int red = ip[0], green = ip[1], blue = ip[2], alpha = ip[3];
         const double luminance = (0.2126 * red + 0.7152 * green + 0.0722 * blue) / 255.0;
         const double material = std::pow(std::clamp(luminance, 0.0, 1.0), 1.35);
         const double normalX = targetW <= 1 ? 0.0 : 2.0 * x / (targetW - 1.0) - 1.0;
         const double bulbFalloff = 0.68 + 0.32 * std::max(0.0, 1.0 - std::sqrt(normalX * normalX * 0.38 + normalY * normalY * 0.72));
         const double addedLight = material * bulbFalloff * lightAmount * 0.54;
         op[0] = ClampByteD(red * drumShade * 0.67 + 255.0 * addedLight);
         op[1] = ClampByteD(green * drumShade * 0.67 + 255.0 * addedLight);
         op[2] = ClampByteD(blue * drumShade * 0.67 + 255.0 * addedLight);
         op[3] = static_cast<uint8_t>(alpha);
      }
   }
   // Side shading: vertical black lines fading inwards from both edges
   const int sideWidth = std::min(std::max(1, targetW / 3), 12 + RoundEven(8.0 * depthAmount));
   for (int i = 0; i < sideWidth; i++)
   {
      const int alpha = std::clamp(RoundEven(105.0 * depthAmount * (1.0 - i / static_cast<double>(sideWidth))), 0, 255);
      for (int y = 0; y < targetH; y++)
      {
         BlendOver(&result.px[(static_cast<size_t>(y) * targetW + i) * 4], 0, 0, 0, alpha);
         if (targetW - 1 - i != i)
            BlendOver(&result.px[(static_cast<size_t>(y) * targetW + (targetW - 1 - i)) * 4], 0, 0, 0, alpha);
      }
   }
   ApplyTemperatureGrade(result, temperature);
   return result;
}

// Aliased wide line (GDI+ pen, no anti aliasing): pixels whose center is within half the width
static void DrawWideLine(ProImage& img, double x1, double y1, double x2, double y2, double width, int r, int g, int b, int a)
{
   const double dx = x2 - x1, dy = y2 - y1;
   const double len2 = dx * dx + dy * dy;
   const double half = width / 2.0;
   const int minX = std::max(0, static_cast<int>(std::floor(std::min(x1, x2) - half))), maxX = std::min(img.w - 1, static_cast<int>(std::ceil(std::max(x1, x2) + half)));
   const int minY = std::max(0, static_cast<int>(std::floor(std::min(y1, y2) - half))), maxY = std::min(img.h - 1, static_cast<int>(std::ceil(std::max(y1, y2) + half)));
   for (int y = minY; y <= maxY; y++)
      for (int x = minX; x <= maxX; x++)
      {
         const double px = x + 0.5 - x1, py = y + 0.5 - y1;
         const double t = len2 > 0.0 ? (px * dx + py * dy) / len2 : 0.0;
         if (t < 0.0 || t > 1.0)
            continue;
         const double ex = px - t * dx, ey = py - t * dy;
         if (ex * ex + ey * ey <= half * half)
            BlendOver(&img.px[(static_cast<size_t>(y) * img.w + x) * 4], r, g, b, a);
      }
}

// Reel3DEffect.CreateWindowOverlay / DrawWindowOverlay
ProImage ProReel3DWindowOverlay(int displayW, int displayH, int depth, int glassReflection)
{
   ProImage overlay;
   if (displayW <= 2 || displayH <= 2)
      return overlay;
   overlay.Allocate(displayW, displayH);
   depth = std::clamp(depth, 0, 200);
   glassReflection = std::clamp(glassReflection, 0, 200);
   const double depthAmount = depth / 100.0;
   int insetWidth = 18 + RoundEven(7.0 * depthAmount);
   insetWidth = std::min(insetWidth, std::max(1, std::min(displayW, displayH) / 2 - 1));
   for (int i = 0; i < insetWidth; i++)
   {
      const int alpha = std::clamp(RoundEven(150.0 * depthAmount * (1.0 - i / static_cast<double>(insetWidth))), 0, 255);
      const int left = i, top = i;
      const int right = i + std::max(1, displayW - 1 - i * 2), bottom = i + std::max(1, displayH - 1 - i * 2);
      for (int x = left; x <= right; x++)
         for (const int y : { top, bottom })
            if (x >= 0 && x < displayW && y >= 0 && y < displayH)
               BlendOver(&overlay.px[(static_cast<size_t>(y) * displayW + x) * 4], 0, 0, 0, alpha);
      for (int y = top + 1; y < bottom; y++)
         for (const int x : { left, right })
            if (x >= 0 && x < displayW && y >= 0 && y < displayH)
               BlendOver(&overlay.px[(static_cast<size_t>(y) * displayW + x) * 4], 0, 0, 0, alpha);
   }
   const int reflectionAlpha = std::clamp(RoundEven(150.0 * glassReflection / 100.0), 0, 255);
   if (reflectionAlpha <= 0)
      return overlay;
   const int sx = 16, sy = 8, sw = std::max(2, displayW - 32), sh = std::max(16, displayH / 3);
   for (int y = std::max(0, sy); y < std::min(displayH, sy + sh); y++)
   {
      const double t = (y + 0.5 - sy) / sh;
      const double a = reflectionAlpha * (1.0 - t);
      for (int x = std::max(0, sx); x < std::min(displayW, sx + sw); x++)
         BlendOver(&overlay.px[(static_cast<size_t>(y) * displayW + x) * 4], 235, 248, 255, a);
   }
   DrawWideLine(overlay, 35, 19, displayW - 85, 7, 2.5, 255, 255, 255, reflectionAlpha);
   DrawWideLine(overlay, displayW / 3, 10, displayW / 2, displayH / 3, 5.0, 210, 238, 255, reflectionAlpha / 2);
   return overlay;
}

ProQuad ProScoreTransform::DisplayQuad(float x, float y, float w, float h) const
{
   const float centerX = x + w / 2.f, centerY = y + h / 2.f;
   const float halfWidth = w / 2.f, halfHeight = h / 2.f;
   const float depth = std::clamp(perspectiveDepth, -1.f, 1.f);
   const float amount = std::abs(depth);
   const float projectedHalfWidth = halfWidth * (1.f - 0.34f * amount);
   const float nearScale = 1.f + 0.06f * amount;
   const float farScale = 1.f - 0.58f * amount;
   const float leftScale = (depth >= 0.f ? nearScale : farScale) * std::clamp(perspectiveLeftScale, 0.2f, 2.5f);
   const float rightScale = (depth >= 0.f ? farScale : nearScale) * std::clamp(perspectiveRightScale, 0.2f, 2.5f);
   ProQuad points { ProPoint { centerX - projectedHalfWidth, centerY - halfHeight * leftScale }, ProPoint { centerX + projectedHalfWidth, centerY - halfHeight * rightScale },
      ProPoint { centerX + projectedHalfWidth, centerY + halfHeight * rightScale }, ProPoint { centerX - projectedHalfWidth, centerY + halfHeight * leftScale } };
   if (std::abs(rotationAngle) > 0.001f)
   {
      const double radians = rotationAngle * 3.14159265358979323846 / 180.0;
      for (ProPoint& p : points)
      {
         const double dx = p.x - centerX, dy = p.y - centerY;
         p = ProPoint { static_cast<float>(centerX + dx * std::cos(radians) - dy * std::sin(radians)), static_cast<float>(centerY + dx * std::sin(radians) + dy * std::cos(radians)) };
      }
   }
   return points;
}

static ProPoint LerpPoint(const ProPoint& a, const ProPoint& b, float t) { return { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t }; }
static ProPoint BilinearPoint(const ProQuad& q, float u, float v) { return LerpPoint(LerpPoint(q[0], q[1], u), LerpPoint(q[3], q[2], u), v); }

ProQuad ProSubQuad(const ProQuad& quad, float u0, float u1, float v0, float v1)
{
   return { BilinearPoint(quad, u0, v0), BilinearPoint(quad, u1, v0), BilinearPoint(quad, u1, v1), BilinearPoint(quad, u0, v1) };
}

void ProContentSize(const ProQuad& q, int& w, int& h)
{
   const auto dist = [](const ProPoint& a, const ProPoint& b) { return std::sqrt(static_cast<double>(b.x - a.x) * (b.x - a.x) + static_cast<double>(b.y - a.y) * (b.y - a.y)); };
   w = std::max(1, RoundEven(dist(q[0], q[1])));
   h = std::max(1, RoundEven(dist(q[0], q[3])));
}

// Inverse of P(u, v) = q0 + (q1 - q0) u + (q3 - q0) v + (q0 - q1 + q2 - q3) u v
static bool InverseBilinear(const ProQuad& q, double px, double py, double& u, double& v)
{
   const double ex = q[1].x - q[0].x, ey = q[1].y - q[0].y;
   const double fx = q[3].x - q[0].x, fy = q[3].y - q[0].y;
   const double gx = q[0].x - q[1].x + q[2].x - q[3].x, gy = q[0].y - q[1].y + q[2].y - q[3].y;
   const double hx = px - q[0].x, hy = py - q[0].y;
   const auto cross = [](double ax, double ay, double bx, double by) { return ax * by - ay * bx; };
   const double k2 = cross(gx, gy, fx, fy);
   const double k1 = cross(ex, ey, fx, fy) + cross(hx, hy, gx, gy);
   const double k0 = cross(hx, hy, ex, ey);
   if (std::abs(k2) < 1e-9)
   {
      if (std::abs(k1) < 1e-12)
         return false;
      v = -k0 / k1;
   }
   else
   {
      double disc = k1 * k1 - 4.0 * k0 * k2;
      if (disc < 0.0)
         return false;
      disc = std::sqrt(disc);
      v = (-k1 - disc) / (2.0 * k2);
      if (v < 0.0 || v > 1.0)
         v = (-k1 + disc) / (2.0 * k2);
   }
   const double denomX = ex + gx * v, denomY = ey + gy * v;
   if (std::abs(denomX) > std::abs(denomY))
      u = (hx - fx * v) / denomX;
   else if (std::abs(denomY) > 1e-12)
      u = (hy - fy * v) / denomY;
   else
      return false;
   return true;
}

void ProWarp(const ProImage& flat, const ProQuad& quad, int& bx, int& by, ProImage& out)
{
   float minX = quad[0].x, maxX = quad[0].x, minY = quad[0].y, maxY = quad[0].y;
   for (const ProPoint& p : quad)
   {
      minX = std::min(minX, p.x);
      maxX = std::max(maxX, p.x);
      minY = std::min(minY, p.y);
      maxY = std::max(maxY, p.y);
   }
   bx = static_cast<int>(std::floor(minX));
   by = static_cast<int>(std::floor(minY));
   const int w = std::max(1, static_cast<int>(std::ceil(maxX)) - bx), h = std::max(1, static_cast<int>(std::ceil(maxY)) - by);
   out.Allocate(w, h);
   if (flat.Empty())
      return;
   int cw, ch;
   ProContentSize(quad, cw, ch);
   const double fu = static_cast<double>(flat.w) / cw, fv = static_cast<double>(flat.h) / ch;
   for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++)
      {
         double u, v;
         if (!InverseBilinear(quad, bx + x + 0.5, by + y + 0.5, u, v) || u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0)
            continue;
         double s[4];
         Sample(flat, u * flat.w, v * flat.h, fu, fv, false, s);
         uint8_t* d = &out.px[(static_cast<size_t>(y) * w + x) * 4];
         d[0] = ClampByteD(s[0]);
         d[1] = ClampByteD(s[1]);
         d[2] = ClampByteD(s[2]);
         d[3] = ClampByteD(s[3]);
      }
}

}
