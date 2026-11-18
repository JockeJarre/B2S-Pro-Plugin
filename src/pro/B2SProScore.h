// license:GPLv3+

// B2S Pro: score display effects on the CPU, ports of the B2S Pro Server's ReelImageSizing (Fit,
// StretchToWidth), Reel3DEffect (RenderReel, CreateWindowOverlay), ScoreTransform (DisplayQuad,
// SubQuad, DrawWarped) and ScoreTransformState (ContentSize). Images are straight alpha RGBA.

#pragma once

#include "common.h"
#include "B2SProData.h"

#include <array>
#include <vector>

namespace B2S {

struct ProImage
{
   vector<uint8_t> px; // RGBA, straight alpha
   int w = 0;
   int h = 0;
   bool Empty() const { return w <= 0 || h <= 0; }
   void Allocate(int width, int height)
   {
      w = width;
      h = height;
      px.assign(static_cast<size_t>(std::max(0, w)) * static_cast<size_t>(std::max(0, h)) * 4, 0);
   }
};

using ProQuad = std::array<ProPoint, 4>; // Top-left, top-right, bottom-right, bottom-left

bool ProReadTexture(VPXTexture tex, ProImage& out);

// Draws src scaled into the rectangle (rx, ry, rw, rh) of dst. copy = SourceCopy, else SourceOver
void ProDrawScaled(ProImage& dst, const ProImage& src, double rx, double ry, double rw, double rh, bool copy);
// Same with a source sub-rectangle (sx, sy, sw, sh)
void ProDrawScaled(ProImage& dst, const ProImage& src, double sx, double sy, double sw, double sh, double rx, double ry, double rw, double rh, bool copy);

// ReelImageSizing
ProImage ProReelFit(const ProImage& source, int targetW, int targetH);
ProImage ProReelStretchToWidth(const ProImage& source, int targetW, int targetH, int contentWidth);

// Reel3DEffect
ProImage ProReel3DRender(const ProImage& source, int targetW, int targetH, int brightness, int temperature, int depth);
// glassStyle: 0 the original reflection lines, 1 soft wide, 2 soft streaks (ReelGlass)
// dividerCenters: B2S Pro reel divider centres in display pixels (nullptr or dividerWidth 0 = none)
ProImage ProReel3DWindowOverlay(int displayW, int displayH, int depth, int glassReflection, int glassStyle, const std::vector<double>* dividerCenters = nullptr, int dividerWidth = 0);
// Realistic reel style (ReelRealistic): numbers[0..4] = the strip at offsets -2..+2 around the shown
// number (+1 = the next number, above; null = paper); roll -1..1 toward the next (+) / previous (-)
ProImage ProReelRealisticRender(const ProImage* const numbers[5], int w, int h, bool keepAspect, int digitWidth,
   int brightness, int temperature, int depth, int curve, int shadow, double roll);
// ReelRealistic.StepMilliseconds: the time one reel step takes
double ProReelStepMs(int rollingIntervalMs, int intermediates);

// ScoreTransform
struct ProScoreTransform
{
   float rotationAngle = 0.f;
   float perspectiveDepth = 0.f;
   float perspectiveLeftScale = 1.f;
   float perspectiveRightScale = 1.f;
   bool active = false; // false when the score is drawn straight on
   ProQuad DisplayQuad(float x, float y, float w, float h) const;
};
ProQuad ProSubQuad(const ProQuad& quad, float u0, float u1, float v0, float v1);
void ProContentSize(const ProQuad& quad, int& w, int& h);

// DrawWarped: maps the flat image onto the quad. Returns the quad's bounding box and its pixels
void ProWarp(const ProImage& flat, const ProQuad& quad, int& bx, int& by, ProImage& out);

}
