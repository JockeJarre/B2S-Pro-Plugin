// license:GPLv3+

// B2S Pro: dynamic front lighting, a port of formBackglass.DrawDynamicFrontLight and
// DynamicArtworkLighting (B2S Pro Server). A dynamic light lights the snippets ranked below it
// through its exported light field; the lit pixels are computed on the CPU and drawn as a texture.

#pragma once

#include "common.h"
#include "B2SDataModel.h"

#include <map>
#include <memory>
#include <vector>

namespace B2S {

class B2SProDynamicLighting final
{
public:
   B2SProDynamicLighting() = default;
   ~B2SProDynamicLighting();

   // Draws what the light adds over the snippets below it (alpha is the light's brightness)
   void DrawLight(VPXRenderContext2D* ctx, const B2SBulb& light, const vector<std::unique_ptr<B2SBulb>>& candidates, B2SDualMode dualMode, float alpha);

   // Snippet frame sampling, shared with other B2S Pro CPU paths: current frame of a snippet
   // (its image placed, and turned for native rotation, as it is drawn) cropped to an area
   struct Rgba
   {
      vector<uint8_t> px; // RGBA, straight alpha
      int w = 0;
      int h = 0;
   };
   static bool ReadTexture(VPXTexture tex, Rgba& out);
   static void SnippetFrameCrop(const B2SBulb& snippet, const Rgba& image, int ox, int oy, int ow, int oh, vector<uint8_t>& out);

private:
   struct Field
   {
      vector<uint8_t> alpha;
      int w = 0;
      int h = 0;
      bool loaded = false;
   };
   std::map<const B2SBulb*, Field> m_fields;
   std::map<const B2SBulb*, Rgba> m_images; // Decoded snippet images

   struct Lit
   {
      VPXTexture tex = nullptr;
      // Inputs of the cached result
      VPXTexture image = nullptr;
      int sx = 0, sy = 0, sw = 0, sh = 0;
      float angle = 0.f, pivotX = 0.f, pivotY = 0.f;
      int ox = 0, oy = 0, ow = 0, oh = 0;
      bool valid = false;
   };
   std::map<std::pair<const B2SBulb*, const B2SBulb*>, Lit> m_lit;

   const Field& GetField(const B2SBulb& light);
   const Rgba* GetImage(const B2SBulb& snippet);
};

// DynamicArtworkLighting.RenderWithWorkspace on straight RGBA pixels (w x h), field = light field alpha (w x h)
void ProRenderDynamicLighting(const uint8_t* source, const uint8_t* field, int w, int h, int intensity, int lightR, int lightG, int lightB, int flasherStyle, int saturation,
   int highlightProtection, int darkAreaLift, int transmissionContrast, int lightTemperature, vector<uint8_t>& output);

}
