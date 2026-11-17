// license:GPLv3+

#include "common.h"

#include "B2SRenderer.h"
#include "B2SServer.h"

#include "pinmame/PinMAMEPlugin.h"
#include "pro/B2SProData.h"
#include "pro/B2SProMotion.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <climits>
#include <cstring>

namespace B2S {

// sRGB to linear conversion (duplicate from the main VPX codebase, see utils/color.h)
static inline float InvsRGB(const float x) { return (x <= 0.04045f) ? (x * (float)(1.0 / 12.92)) : (powf(x * (float)(1.0 / 1.055) + (float)(0.055 / 1.055), 2.4f)); }

MSGPI_BOOL_VAL_SETTING(showGrillProp, "ShowGrill", "Show Grill", "Show Grill", true, false);
MSGPI_BOOL_VAL_SETTING(upgradedSegRenderingProp, "UpgradedSegmentRendering", "Upgraded segment rendering",
   "Render LED score displays with the upgraded segment renderer (turn off for the original B2S segment rendering)", true, false);

// The backglass and score view images keep their shape in their window: the drawing surface is padded to
// the window shape, the images are scaled by a uniform factor and the leftover area is kept black. The
// setting is a percentage of this fit: 0 stretches the images over the whole window (plain B2S, no black
// area), 100 keeps the aspect ratio fully, in between the black area and the leftover distortion are
// applied that part of the way
MSGPI_INT_VAL_SETTING(keepAspectRatioProp, "KeepAspectRatioPercent", "Keep aspect ratio (%)",
   "Fit the backglass and score view images in their window at their authored aspect ratio, leaving the "
   "leftover area black: 100 keeps the aspect ratio fully (the image is never distorted, the black area is "
   "at its widest), 0 turns it off and stretches the images over the whole window, in between the padding "
   "and the leftover distortion are applied that part of the way (50: half the black area, half the "
   "distortion)", true, 0, 100, 0);

// The black area of the fit goes top and bottom (centered) or all on top (content on the floor)
MSGPI_BOOL_VAL_SETTING(centerVerticallyProp, "CenterVertically", "Center vertically",
   "Center the images vertically in the black area left by 'Keep aspect ratio' (turn off to put them on the "
   "floor). At 0 the black area is empty, at 50 its half width goes above the image", true, false); // B2S Pro: default off, matching the B2S Server look

static const char* dualModeValues[] = { "Authentic", "Fantasy" };
MSGPI_ENUM_VAL_SETTING(dualModeProp, "DualMode", "Dual Mode", "Active mode for dual backglasses (authentic or fantasy)", true, 1, 2, dualModeValues, 1);

B2SRenderer::B2SRenderer(const MsgPluginAPI* const msgApi, const VPXPluginAPI* const vpxApi, const unsigned int endpointId, std::shared_ptr<B2STable> b2s)
   : m_b2s(b2s)
   , m_msgApi(msgApi)
   , m_vpxApi(vpxApi)
   , m_endpointId(endpointId)
   , m_resURIResolver(*msgApi, endpointId, true, false, false)
   , m_pinmameControllers(
        msgApi, endpointId, CTLPI_CONTROLLERS_GET_MSG, CTLPI_CONTROLLERS_ON_CHG_MSG,
        [](std::vector<ControllerDef>& items)
        {
           const string pinmamePrefix(PMPI_GAMEID_PREFIX);
           std::erase_if(items, [&pinmamePrefix](const ControllerDef& src) { return !string(src.gameId).starts_with(pinmamePrefix); });
        },
        nullptr,
        [this]()
        {
           m_stateSources.Refresh();
           m_segSources.Refresh();
        })
   , m_segSources(
        msgApi, endpointId, CTLPI_SEG_GET_SRC_MSG, CTLPI_SEG_ON_SRC_CHG_MSG,
        [this](std::vector<SegSrcId>& items)
        {
           m_pinmameControllers.With(
              [&items](const std::vector<ControllerDef>& controllers)
              {
                 std::erase_if(items,
                    [&controllers](const SegSrcId& source)
                    {
                       return std::find_if(controllers.begin(), controllers.end(), [source](const ControllerDef& ctrl) { return ctrl.endpointId == source.id.endpointId; })
                          == controllers.end();
                    });
              });
        },
        nullptr, nullptr)
   , m_stateSources(
        msgApi, endpointId, CTLPI_STATE_GET_SRC_MSG, CTLPI_STATE_ON_SRC_CHG_MSG,
        [this](std::vector<StateSrcId>& items)
        {
           m_pinmameControllers.With(
              [&items](const std::vector<ControllerDef>& controllers)
              {
                 std::erase_if(items,
                    [&controllers](const StateSrcId& source)
                    {
                       return std::find_if(controllers.begin(), controllers.end(), [source](const ControllerDef& ctrl) { return ctrl.endpointId == source.id.endpointId; })
                          == controllers.end();
                    });
              });
        },
        [this]() { m_stateSources.With([this](const std::vector<StateSrcId>& items) { OnStateSrcChanged({}); }); },
        [this]() { m_stateSources.With([this](const std::vector<StateSrcId>& items) { OnStateSrcChanged(items); }); })
{
   const VPXTextureInfo* dmdTexInfo = nullptr;
   if (m_b2s->m_dmdImage.m_image)
      dmdTexInfo = GetTextureInfo(m_b2s->m_dmdImage.m_image);
   m_dmdWidth = dmdTexInfo ? static_cast<float>(dmdTexInfo->width) : 1024.f;
   m_dmdHeight = dmdTexInfo ? static_cast<float>(dmdTexInfo->height) : 768.f;

   if (m_b2s->m_unifiedLayerDeck)
      m_proRuntime = std::make_unique<B2SProRuntime>(*m_b2s);

   m_segSources.Subscribe();
   m_stateSources.Subscribe();
   m_pinmameControllers.Subscribe();
}

B2SRenderer::~B2SRenderer()
{
   for (auto& [key, mask] : m_proBallMasks)
      if (mask.frame != nullptr)
         DeleteTexture(mask.frame);
   for (auto& [key, digit] : m_proDigits)
      if (digit.tex != nullptr)
         DeleteTexture(digit.tex);
   for (auto& [key, gap] : m_proReelGaps)
      if (gap.tex != nullptr)
         DeleteTexture(gap.tex);
   for (auto& [key, state] : m_legacySegDisplays)
      for (VPXTexture tex : state.warped)
         if (tex != nullptr)
            DeleteTexture(tex);
   if (m_backingTex != nullptr)
      DeleteTexture(m_backingTex);
   m_segSources.Unsubscribe();
   m_stateSources.Unsubscribe();
   m_pinmameControllers.Unsubscribe();
}

void B2SRenderer::RegisterSettings(const MsgPluginAPI* const msgApi, unsigned int endpointId)
{
   msgApi->RegisterSetting(endpointId, &showGrillProp);
   msgApi->RegisterSetting(endpointId, &dualModeProp);
   msgApi->RegisterSetting(endpointId, &upgradedSegRenderingProp);
   msgApi->RegisterSetting(endpointId, &keepAspectRatioProp);
   msgApi->RegisterSetting(endpointId, &centerVerticallyProp);
}

namespace
{
   int MSGPIAPI ShowGrillGet(void*) { return showGrillProp_Get(); }
   void MSGPIAPI ShowGrillSet(void*, int v) { showGrillProp_Set(v); }
   int MSGPIAPI DualModeGet(void*) { return dualModeProp_Get(); }
   void MSGPIAPI DualModeSet(void*, int v) { dualModeProp_Set(v); }
   int MSGPIAPI UpgradedSegGet(void*) { return upgradedSegRenderingProp_Get(); }
   void MSGPIAPI UpgradedSegSet(void*, int v) { upgradedSegRenderingProp_Set(v); }
   int MSGPIAPI KeepAspectRatioGet(void*) { return keepAspectRatioProp_Get(); }
   void MSGPIAPI KeepAspectRatioSet(void*, int v) { keepAspectRatioProp_Set(v); }
   int MSGPIAPI CenterVerticallyGet(void*) { return centerVerticallyProp_Get(); }
   void MSGPIAPI CenterVerticallySet(void*, int v) { centerVerticallyProp_Set(v); }
}

int B2SRenderer::FillSettingsItems(PagedUIItemDef* items)
{
   items[0] = {};
   items[0].type = PAGEDUI_ITEM_BOOL;
   items[0].id = "ShowGrill";
   items[0].label = "Show Grill";
   items[0].tooltip = "Show Grill";
   items[0].storage = PAGEDUI_STORAGE_HOST;
   items[0].boolProp = { 0, &ShowGrillGet, &ShowGrillSet };
   items[1] = {};
   items[1].type = PAGEDUI_ITEM_ENUM;
   items[1].id = "DualMode";
   items[1].label = "Dual Mode";
   items[1].tooltip = "Active mode for dual backglasses (authentic or fantasy)";
   items[1].storage = PAGEDUI_STORAGE_HOST;
   items[1].enumProp = { 1, 2, dualModeValues, 1, &DualModeGet, &DualModeSet };
   items[2] = {};
   items[2].type = PAGEDUI_ITEM_BOOL;
   items[2].id = "UpgradedSegmentRendering";
   items[2].label = "Upgraded segment rendering";
   items[2].tooltip = "Render LED score displays with the upgraded segment renderer (turn off for the original B2S segment rendering)";
   items[2].storage = PAGEDUI_STORAGE_HOST;
   items[2].boolProp = { 1, &UpgradedSegGet, &UpgradedSegSet };
   items[3] = {};
   items[3].type = PAGEDUI_ITEM_INT;
   items[3].id = "KeepAspectRatioPercent";
   items[3].label = "Keep aspect ratio";
   items[3].tooltip = "Fit the backglass and score view images in their window at their authored aspect ratio, leaving the "
      "leftover area black: 100 keeps the aspect ratio fully (the image is never distorted, the black area is at its widest), "
      "0 turns it off and stretches the images over the whole window, in between the black area and the leftover distortion "
      "are applied that part of the way (50: half the black area, half the distortion)";
   items[3].storage = PAGEDUI_STORAGE_HOST;
   items[3].intProp = { 0, 100, 0, "%d %%", &KeepAspectRatioGet, &KeepAspectRatioSet };
   items[4] = {};
   items[4].type = PAGEDUI_ITEM_BOOL;
   items[4].id = "CenterVertically";
   items[4].label = "Center vertically";
   items[4].tooltip = "Center the images vertically in the black area left by 'Keep aspect ratio' (turn off to put them on the floor). At 0 the black area is empty, at 50 its half width goes above the image";
   items[4].storage = PAGEDUI_STORAGE_HOST;
   items[4].boolProp = { 1, &CenterVerticallyGet, &CenterVerticallySet };
   return 5;
}

// KeepAspectRatio rendering: the host stretches the source surface (the fictitious screen the plugin draws
// on) over the whole window, each axis on its own. The surface is therefore padded to the window shape, so
// that both axes scale by the very same factor: the authored content keeps its shape (a round bulb stays
// round, the image is not stretched) and the padding stays black. Nothing else has to be known by the
// drawing code, it keeps working in the image authored units.
//
// The setting is a percentage of this fit: the padding the full fit needs is applied that part of the way,
// which takes both axes scale factors from the window/image ratio to 1 and the black area from nothing to
// its full width, the two ends met at 0 (plain stretch) and 100 (aspect ratio fully kept).
void B2SRenderer::FitSurfaceToWindow(VPXRenderContext2D* ctx, const float contentW, const float contentH)
{
   m_padW = m_padH = 0.f;
   const int keep = keepAspectRatioProp_Get();
   if (keep <= 0 || contentW <= 0.f || contentH <= 0.f || ctx->outWidth <= 0.f || ctx->outHeight <= 0.f)
   {
      ctx->srcWidth = contentW;
      ctx->srcHeight = contentH;
      m_contentW = contentW;
      m_contentH = contentH;
      m_layoutOY = 0.f;
      m_layoutOX = 0.f;
      return;
   }

   const float wndAspect = ctx->outWidth / ctx->outHeight;
   const bool wideContent = (contentW / contentH > wndAspect);
   const float fitW = wideContent ? contentW : contentH * wndAspect;
   const float fitH = wideContent ? contentW / wndAspect : contentH;
   const float part = (keep >= 100) ? 1.f : (0.01f * static_cast<float>(keep));
   ctx->srcWidth = m_padW = contentW + part * (fitW - contentW);
   ctx->srcHeight = m_padH = contentH + part * (fitH - contentH);
   m_contentW = contentW;
   m_contentH = contentH;
   // Vertical placement of the content in the surface: centered, or stuck on the floor. The content is drawn
   // from the top of the surface, so the placement is the black area taken above it
   m_layoutOY = centerVerticallyProp_Get() ? 0.5f * (m_padH - contentH) : m_padH - contentH;
   // Horizontal placement is centered, there is nothing to choose: the image is centered on its axis of
   // authoring, the only case where a horizontal black area shows is a window wider than the image
   m_layoutOX = 0.5f * (m_padW - contentW);
}

// The padding is painted over once the frame is drawn: animations, illuminations and score displays are
// authored on the image and may spill in the area kept for the borders, which stays black
void B2SRenderer::RenderAspectPadding(VPXRenderContext2D* ctx)
{
   const float left = m_layoutOX;
   const float top = m_layoutOY;
   const float bottom = m_padH - m_layoutOY - m_contentH;
   const float right = m_padW - m_layoutOX - m_contentW;
   if (left <= 0.f && right <= 0.f && top <= 0.f && bottom <= 0.f)
      return;

   if (m_blackTexture == nullptr)
   {
      static const uint8_t black[4] = { 0, 0, 0, 255 };
      m_vpxApi->UpdateTexture(&m_blackTexture, 1, 1, VPXTextureFormat::VPXTEXFMT_sRGBA8, black);
      if (m_blackTexture == nullptr)
         return; // Texture upload failed, keep the padding as it is
   }

   const auto drawBar = [this, ctx](const float x, const float y, const float w, const float h)
   {
      ctx->DrawImage(ctx, m_blackTexture, 0.f, 0.f, 0.f, 1.f, // Opaque black
         0.f, 0.f, 1.f, 1.f, // The 1x1 texture, fully sampled
         0.f, 0.f, 0.f, // No pivot, no rotation
         x, y, w, h);
   };

   if (top > 0.f)
      drawBar(0.f, 0.f, ctx->srcWidth, top);
   if (bottom > 0.f)
      drawBar(0.f, m_layoutOY + m_contentH, ctx->srcWidth, bottom);
   if (left > 0.f)
      drawBar(0.f, m_layoutOY, left, m_contentH);
   if (right > 0.f)
      drawBar(m_layoutOX + m_contentW, m_layoutOY, right, m_contentH);
}

void B2SRenderer::OnStateSrcChanged(const std::vector<StateSrcId>& items)
{
   if (m_b2s->m_backglassOnImage.m_image)
      m_b2s->m_backglassOnImage.m_romUpdater = []() { /* No ROM source */ };
   for (auto* bulbs : { &m_b2s->m_backglassIlluminations, &m_b2s->m_dmdIlluminations })
      for (auto& bulb : *bulbs)
         bulb->m_romUpdater = []() { /* No ROM source */ };


   // The reference forces TopRomInverted to False for the BackglassOnImage channel
   if (m_b2s->m_backglassOnImage.m_image)
      m_b2s->m_backglassOnImage.m_romUpdater
         = ResolveRomPropUpdater(items, &m_b2s->m_backglassOnImage.m_romProbe, m_b2s->m_backglassOnImage.m_romIdType, m_b2s->m_backglassOnImage.m_romId, false);

   // The reference only defines a matching value (B2SValue as RomIDValue) on B2SID bulbs; ROM events and
   // script writes share the same lamp channel there, so the value filter applies to both paths
   for (auto* bulbs : { &m_b2s->m_backglassIlluminations, &m_b2s->m_dmdIlluminations })
      for (auto& bulb : *bulbs)
      {
         const int romValue = bulb->m_b2sId >= 0 ? bulb->m_b2sValue : 0;
         switch (bulb->m_snippitType)
         {
         case B2SSnippitType::StandardImage: bulb->m_romUpdater = ResolveRomPropUpdater(items, &bulb->m_romProbe, bulb->m_romIdType, bulb->m_romId, bulb->m_romInverted, romValue); break;
         case B2SSnippitType::MechRotatingImage: bulb->m_romUpdater = ResolveRomPropUpdater(items, &bulb->m_mechProbe, bulb->m_romIdType, bulb->m_romId); break;
         case B2SSnippitType::SelfRotatingImage: bulb->m_romUpdater = ResolveRomPropUpdater(items, &bulb->m_romOn, bulb->m_romIdType, bulb->m_romId, bulb->m_romInverted, romValue); break;
         }
      }

   // Reel illumination lamps (ReelIlluB2SID is always a lamp in the reference implementation)
   for (auto* scores : { &m_b2s->m_backglassScores, &m_b2s->m_dmdScores })
      for (auto& score : scores->m_scores)
         score.m_reelIlluUpdater = score.m_reelIlluB2SID > 0 ? ResolveRomPropUpdater(items, &score.m_reelIllu, B2SRomIDType::Lamp, score.m_reelIlluB2SID) : []() { };

   // B2S Pro: rotation and pivot triggers read the raw state (inversion is applied by the runtime)
   if (m_proRuntime)
      m_proRuntime->BindTriggers([this, &items](B2SRomIDType romIdType, int romId, float* target) { return ResolveRomPropUpdater(items, target, romIdType, romId); });

   // Rebind the ROM event triggers of all animations (IDJoin)
   const auto resolver = [this, &items](B2SRomIDType romIdType, int romId, bool inverted, float* target) { return ResolveRomPropUpdater(items, target, romIdType, romId, inverted); };
   for (auto& animation : m_b2s->m_backglassAnimations)
      animation.BindRomTriggers(resolver);
   for (auto& animation : m_b2s->m_dmdAnimations)
      animation.BindRomTriggers(resolver);
}

std::function<void()> B2SRenderer::ResolveRomPropUpdater(const std::vector<StateSrcId>& items, float* value, const B2SRomIDType romIdType, const int romId, const bool romInverted, const int romValue) const
{
   int groupId;
   switch (romIdType)
   {
   case B2SRomIDType::Solenoid: groupId = PMPI_GROUP_SOLENOID; break;
   case B2SRomIDType::GIString: groupId = PMPI_GROUP_GI; break;
   case B2SRomIDType::Lamp: groupId = PMPI_GROUP_LAMP; break;
   case B2SRomIDType::Mech: groupId = PMPI_GROUP_MECH; break;
   case B2SRomIDType::Switch: groupId = PMPI_GROUP_SWITCH; break; // B2S Pro: polled like the reference polls VPinMAME.Switch
   default: return []() { /* No ROM source */ };
   }
   for (const StateSrcId& src : items)
   {
      if (src.id.resId != groupId)
         continue;
      for (unsigned int i = 0; i < src.nStates; i++)
      {
         const auto& def = src.stateDefs[i];
         // B2S Pro: switch states may use any numeric format (0 open, non 0 closed)
         if (romIdType == B2SRomIDType::Switch && def.mappingId == romId && def.dataFormat != CTLPI_STATE_FORMAT_FLOAT && def.dataFormat != CTLPI_STATE_FORMAT_STRING && def.GetState)
         {
            return [this, value, def, romInverted]()
            {
               uint8_t raw[16] = {};
               m_stateSources.With([&def, &raw](const std::vector<StateSrcId>&) { def.GetState(def.callContext, raw); });
               bool closed = false;
               switch (def.dataFormat)
               {
               case CTLPI_STATE_FORMAT_UINT8:
               case CTLPI_STATE_FORMAT_INT8: closed = raw[0] != 0; break;
               case CTLPI_STATE_FORMAT_UINT16:
               case CTLPI_STATE_FORMAT_INT16: closed = (raw[0] | raw[1]) != 0; break;
               case CTLPI_STATE_FORMAT_UINT32:
               case CTLPI_STATE_FORMAT_INT32: closed = (raw[0] | raw[1] | raw[2] | raw[3]) != 0; break;
               case CTLPI_STATE_FORMAT_DOUBLE: { double d; memcpy(&d, raw, sizeof(d)); closed = d != 0.0; break; }
               default: { uint64_t q; memcpy(&q, raw, sizeof(q)); closed = q != 0; break; }
               }
               *value = (closed != romInverted) ? 1.f : 0.f;
            };
         }
         if (def.mappingId == romId && def.dataFormat == CTLPI_STATE_FORMAT_FLOAT && def.GetState)
         {
            return [this, value, def, romInverted, romValue]()
            {
               m_stateSources.With([this, value, &def](const std::vector<StateSrcId>&) { def.GetState(def.callContext, value); });
               if (romValue > 0)
                  *value = (static_cast<int>(*value) == romValue) ? 1.f : 0.f;
               else if (romInverted)
                  *value = 1.f - *value;
            };
         }
      }
   }
   return []() { /* No ROM source */ };
}

bool B2SRenderer::Render(VPXRenderContext2D* ctx, B2SServer* server)
{
   switch (ctx->window)
   {
   case VPXWindowId::VPXWINDOW_Backglass: return RenderBackglass(ctx, server);
   case VPXWindowId::VPXWINDOW_ScoreView: return RenderScoreView(ctx, server);
   default: return false;
   }
}

B2SDualMode B2SRenderer::ActiveDualMode() const { return m_b2s->m_dualBackglass ? static_cast<B2SDualMode>(dualModeProp_Get()) : B2SDualMode::Both; }

void B2SRenderer::UpdateAnimations(vector<B2SAnimation>& animations, float elapsed, B2SServer* server, B2SDualMode dualMode)
{
   B2SAnimationEffects fx;
   if (server)
      fx = server->GetAnimationEffects();
   fx.dualMode = dualMode;
   fx.randomTrigger = [this, &animations](B2SRomIDType romIdType, int romId, bool start, B2SAnimation*) { OnRandomAnimationTrigger(animations, romIdType, romId, start); };
   for (auto& animation : animations)
   {
      // Animation slow downs from the plugin settings (AllAnimationSlowDown takes precedence over per-animation)
      float scaledElapsed = elapsed;
      if (server)
      {
         int slowDown = server->GetAllAnimationSlowDown();
         if (slowDown <= 1)
            slowDown = server->GetAnimationSlowDown(animation.m_name);
         if (slowDown > 1)
            scaledElapsed /= static_cast<float>(slowDown);
      }
      animation.Update(scaledElapsed, fx);
   }
}

// Script B2SSetData writes in the lamp-id space trigger animations bound to that lamp through IDJoin
// (mirrors the reference's MyB2SSetData which treats the B2SID space as lamp space)
void B2SRenderer::DispatchScriptTriggers(B2SServer* server)
{
   for (const auto& [lampId, value] : server->DrainAnimationTriggers())
      for (auto* animations : { &m_b2s->m_backglassAnimations, &m_b2s->m_dmdAnimations })
         for (auto& animation : *animations)
            for (const auto& trigger : animation.GetRomTriggers())
               if (trigger.romIdType == B2SRomIDType::Lamp && trigger.romId == lampId)
               {
                  const bool start = (value != 0) != trigger.inverted;
                  if (animation.m_randomStart)
                     OnRandomAnimationTrigger(*animations, B2SRomIDType::Lamp, lampId, start);
                  else if (start)
                     animation.Start();
                  else
                     animation.Stop();
                  break;
               }
}

void B2SRenderer::OnRandomAnimationTrigger(vector<B2SAnimation>& animations, B2SRomIDType romIdType, int romId, bool start)
{
   if (start)
   {
      // Pick one animation at random among all the random-start animations sharing this ROM trigger,
      // weighted by RandomQuality (the reference adds the animation once per quality point to the pool)
      vector<B2SAnimation*> pool;
      int totalWeight = 0;
      for (B2SAnimation& animation : animations)
         if (animation.m_randomStart
            && std::ranges::any_of(
               animation.GetRomTriggers(), [romIdType, romId](const B2SAnimation::RomTrigger& trigger) { return trigger.romIdType == romIdType && trigger.romId == romId; }))
         {
            pool.push_back(&animation);
            totalWeight += std::max(1, animation.m_randomQuality);
         }
      const bool anyRunning = std::ranges::any_of(pool, [](const B2SAnimation* animation) { return animation->IsRunning(); });
      if (!anyRunning && !pool.empty())
      {
         int pick = std::rand() % totalWeight;
         for (B2SAnimation* animation : pool)
         {
            pick -= std::max(1, animation->m_randomQuality);
            if (pick < 0)
            {
               m_lastRandomAnimation = animation;
               break;
            }
         }
         m_lastRandomAnimation->Start(false);
      }
   }
   else if (m_lastRandomAnimation != nullptr)
   {
      m_lastRandomAnimation->Stop();
      m_lastRandomAnimation = nullptr;
   }
}

// B2S Pro: the per-bulb work is split into a state update and a draw so that the unified layer
// deck can update every bulb once per frame and draw them interleaved with the canvas and the
// score displays. RenderBulbs keeps the stock order (update then draw, bulb by bulb).
bool B2SRenderer::IsBulbRendered(const B2SBulb& bulb, B2SDualMode dualMode)
{
   if (dualMode != B2SDualMode::Both && bulb.m_dualMode != B2SDualMode::Both && bulb.m_dualMode != dualMode)
      return false; // Dual backglass: only render bulbs matching the active mode
   if (bulb.m_bakedIntoBackground)
      return false; // Drawn as part of the lit background image
   return true;
}

void B2SRenderer::RenderBulbs(VPXRenderContext2D* ctx, const B2SServer* server, const vector<std::unique_ptr<B2SBulb>>& bulbs, float elapsed, B2SDualMode dualMode)
{
   for (const auto& bulb : bulbs)
   {
      if (!IsBulbRendered(*bulb, dualMode))
         continue;
      UpdateBulb(server, *bulb, elapsed);
      DrawBulb(ctx, *bulb);
   }
}

void B2SRenderer::UpdateBulb(const B2SServer* server, B2SBulb& bulbRef, float elapsed)
{
   // B2S Pro: native rotation and pivot pictures take their visibility and angle from the B2S Pro runtime
   if (bulbRef.m_pro && bulbRef.m_pro->rt.driven)
   {
      bulbRef.m_brightness = bulbRef.m_pro->rt.visible ? 1.f : 0.f;
      // VisualRotationAngle: rotation plus the rolling angle of a rolling motion path picture
      bulbRef.m_drawRotation = bulbRef.m_pro->rt.angle + (bulbRef.m_pro->motionPathRollEnabled ? bulbRef.m_pro->rt.motion.rollAngle : 0.f);
      return;
   }
   {
      B2SBulb* const bulb = &bulbRef;
      const bool locked = server && !bulb->m_name.empty() && server->IsIlluminationLocked(bulb->m_name);
      // ROM polling: a ROM state change counts as a write into the same channel as script writes
      // (the reference routes both through the same shared state, the most recent one wins), so the
      // probe keeps updating even while a script write is in control. Probes are suspended while the
      // illumination group is locked by an animation, and the first observation does not count as a
      // change. Mech values are rotation steps, not visibility writes, and never take the channel.
      if (!locked)
      {
         float* const probe = bulb->m_snippitType == B2SSnippitType::MechRotatingImage ? &bulb->m_mechProbe
            : bulb->m_snippitType == B2SSnippitType::SelfRotatingImage                 ? &bulb->m_romOn
                                                                                       : &bulb->m_romProbe;
         const float prev = *probe;
         bulb->m_romUpdater();
         if (server != nullptr && prev >= 0.f && *probe != prev && bulb->m_romIdType != B2SRomIDType::Mech)
            bulb->m_romStamp = server->NextStamp();
         // B2S Pro: a switch light takes the first observed switch state (formBackglass SetTesterSwitchState)
         else if (server != nullptr && bulb->m_pro && bulb->m_romIdType == B2SRomIDType::Switch && prev < 0.f && *probe >= 0.f)
            bulb->m_romStamp = server->NextStamp();
      }
      float state = 0.f;
      bool fromLamp = false;
      const bool scripted = server && server->GetBulbState(*bulb, state, &fromLamp);
      // Script writes on the lamp channel of a rotating image drive its rotation instead of its
      // visibility (reference): for mech images the value selects the rotation step, for self
      // rotating images it toggles the spinning, most recent write between script and ROM wins
      const int lampId = (bulb->m_romIdType == B2SRomIDType::Lamp && bulb->m_romId > 0) ? bulb->m_romId : -1;
      uint64_t lampWriteStamp = 0;
      float lampWriteState = 0.f;
      const bool lampWinsRotation = server != nullptr && !locked && lampId > 0 && server->GetScriptedLampState(lampId, lampWriteState, &lampWriteStamp) && lampWriteStamp > bulb->m_romStamp;
      float rotation = 0.f;
      if (bulb->m_snippitType == B2SSnippitType::MechRotatingImage)
      {
         // Mech rotating images stay visible (the channel value only selects the rotation step);
         // illumination group writes still drive their visibility
         if (scripted && !fromLamp)
            bulb->m_brightness = state;
         if (lampWinsRotation)
            bulb->m_mechRot = lampWriteState;
         else if (bulb->m_mechProbe >= 0.f)
            bulb->m_mechRot = bulb->m_mechProbe;
         rotation = 360.f * (bulb->m_mechRot / static_cast<float>(bulb->m_snippitRotatingSteps));
      }
      else if (bulb->m_snippitType == B2SSnippitType::SelfRotatingImage)
      {
         if (scripted && !fromLamp)
            bulb->m_brightness = state;
         float spin = std::max(0.f, bulb->m_romOn);
         if (lampWinsRotation)
            spin = (bulb->m_b2sId >= 0 && bulb->m_b2sValue > 0) ? (static_cast<int>(lampWriteState) == bulb->m_b2sValue ? 1.f : 0.f)
                                                                : ((lampWriteState != 0.f) != bulb->m_romInverted ? 1.f : 0.f);
         // Rotation only toggles on state transitions so that explicit B2SStartRotation calls keep
         // spinning until B2SStopRotation or a state write (reference behaviour)
         const bool spinOn = spin >= 0.5f;
         if (spinOn != bulb->m_spinDriverOn)
         {
            bulb->m_spinDriverOn = spinOn;
            if (spinOn)
               bulb->StartRotation();
            else
               bulb->StopRotation();
         }
         bulb->UpdateRotation(elapsed);
         rotation = bulb->GetRotationAngle();
      }
      else if (scripted)
         bulb->m_brightness = state;
      else if (bulb->m_romStamp > 0 || (server == nullptr && bulb->m_romProbe >= 0.f))
         // The probe only takes over once a ROM change was actually observed: the first observation
         // is the initial state read, not a write, so InitialState survives until the first event
         bulb->m_brightness = bulb->m_romProbe;
      if (bulb->m_pro)
         UpdateProBulbVisibility(*bulb, elapsed);
      bulb->m_drawRotation = rotation;
   }
}

// B2S Pro: switch light hold and flasher pulse (B2SPictureBox.Visible / formBackglass.SetTesterSwitchState)
void B2SRenderer::UpdateProBulbVisibility(B2SBulb& bulb, float elapsed)
{
   const B2SProBulbData& pro = *bulb.m_pro;
   bulb.m_proClockMs += static_cast<double>(elapsed) * 1000.0;
   if (bulb.m_snippitType != B2SSnippitType::StandardImage)
      return;

   // A switch light stays in its closed state for at least 150 ms; an earlier open is applied after that
   if (bulb.m_romIdType == B2SRomIDType::Switch && bulb.m_romProbe >= 0.f && bulb.m_romStamp > 0)
   {
      const bool closed = (bulb.m_romProbe >= 0.5f) != bulb.m_romInverted;
      if (closed && !bulb.m_switchLastClosed)
         bulb.m_switchHoldUntilMs = bulb.m_proClockMs + 150.0;
      bulb.m_switchLastClosed = closed;
      if (!closed && bulb.m_proClockMs < bulb.m_switchHoldUntilMs && bulb.m_brightness == bulb.m_romProbe)
         bulb.m_brightness = bulb.m_romInverted ? 0.f : 1.f;
   }

   // A flasher pulse shows the light for its duration from each switch-on, whatever the source does
   // in between; once the pulse ends the light stays off until the next switch-on
   if (pro.flasherPulseDuration > 0)
   {
      const bool on = bulb.m_brightness >= 0.5f;
      const bool event = on && (!bulb.m_pulseLastOn || bulb.m_romStamp != bulb.m_pulseLastStamp);
      bulb.m_pulseLastOn = on;
      bulb.m_pulseLastStamp = bulb.m_romStamp;
      if (event)
         bulb.m_pulseUntilMs = bulb.m_proClockMs + static_cast<double>(pro.flasherPulseDuration);
      bulb.m_brightness = (bulb.m_proClockMs < bulb.m_pulseUntilMs) ? 1.f : 0.f;
   }
}

void B2SRenderer::DrawBulb(VPXRenderContext2D* ctx, const B2SBulb& bulbRef)
{
   if (bulbRef.m_pro)
      DrawProBulb(ctx, bulbRef);
   else
      DrawBulbImage(ctx, bulbRef);
}

// B2S Pro: formBackglass.DrawImage / formDMD.DrawIlluminationImage. A hidden picture draws nothing; a
// visible one draws its image (the OffImage instead while the unlit backglass is shown), untinted.
// A dynamic front light on the backglass draws its light over the snippets below it instead (and its
// own image too when it is on a separate light layer).
void B2SRenderer::DrawProBulb(VPXRenderContext2D* ctx, const B2SBulb& bulb)
{
   const B2SProBulbData& pro = *bulb.m_pro;
   // DrawPersistentMotionPathSource: a sequence feeder drawn at the path start, whatever the
   // picture's own visibility
   if (B2SProMotionSystem::IsSourceSeparate(bulb))
   {
      VPXTexture source = B2SProMotionSystem::SourceImage(bulb);
      if (m_drawingBackglass && m_b2s->m_backglassOffImage.m_image && m_backglassDark && bulb.m_offImage)
         source = bulb.m_offImage;
      if (source != nullptr)
      {
         const ProRect r = B2SProMotionSystem::SourceRect(bulb);
         const VPXTextureInfo* const tex = GetTextureInfo(source);
         ctx->DrawImage(ctx, source, 1.f, 1.f, 1.f, 1.f, 0.f, 0.f, static_cast<float>(tex->width), static_cast<float>(tex->height), static_cast<float>(tex->width) * 0.5f,
            static_cast<float>(tex->height) * 0.5f, B2SProMotionSystem::SourceRollAngle(bulb), ContentX(r.x), ContentY(r.y), r.w, r.h);
      }
   }
   if (bulb.m_brightness <= 0.f)
      return;
   if (m_drawingBackglass && pro.dynamicFrontLighting && !bulb.m_isImageSnippit)
   {
      if (pro.separateLightLayer && bulb.m_image)
         DrawProImage(ctx, bulb, bulb.m_image);
      m_proLighting.DrawLight(ctx, bulb, m_b2s->m_backglassIlluminations, ActiveDualMode(), bulb.m_brightness, m_layoutOX, m_layoutOY);
      return;
   }
   VPXTexture texture = pro.rt.motion.image ? pro.rt.motion.image : bulb.m_image;
   if (m_drawingBackglass && m_b2s->m_backglassOffImage.m_image && m_backglassDark && bulb.m_offImage)
      texture = bulb.m_offImage;
   DrawProImage(ctx, bulb, texture);
}

// B2S Pro: CreatePhysicsMaskedFrame. The ball turns inside a fixed selection: its frame (turned
// around its center) is multiplied by the backglass sized selection alpha at the same pixels.
bool B2SRenderer::DrawProMaskedBall(VPXRenderContext2D* ctx, const B2SBulb& bulb, VPXTexture texture)
{
   ProBallMask& mask = m_proBallMasks[&bulb];
   if (!mask.loaded)
   {
      mask.loaded = true;
      vector<uint8_t> png = bulb.m_pro->physicsSelectionMaskPng;
      if (VPXTexture tex = CreateTexture(png.data(), static_cast<int>(png.size())))
      {
         ProImage decoded;
         if (ProReadTexture(tex, decoded))
         {
            mask.w = decoded.w;
            mask.h = decoded.h;
            mask.alpha.resize(static_cast<size_t>(decoded.w) * decoded.h);
            for (size_t i = 0; i < mask.alpha.size(); i++)
               mask.alpha[i] = decoded.px[i * 4 + 3];
         }
         DeleteTexture(tex);
      }
   }
   if (mask.w <= 0 || mask.h <= 0)
      return false;
   B2SProDynamicLighting::Rgba image;
   if (!B2SProDynamicLighting::ReadTexture(texture, image))
      return false;
   const ProMotion& m = bulb.m_pro->rt.motion;
   // Bounds of the turned rectangle (BallRollDestinationPoints)
   const double a = bulb.m_drawRotation * 3.14159265358979323846 / 180.0;
   const double hw = m.w / 2.0, hh = m.h / 2.0, cx = m.x + hw, cy = m.y + hh;
   const double ex = std::abs(hw * std::cos(a)) + std::abs(hh * std::sin(a)), ey = std::abs(hw * std::sin(a)) + std::abs(hh * std::cos(a));
   const int left = static_cast<int>(std::floor(cx - ex)), top = static_cast<int>(std::floor(cy - ey));
   const int fw = std::max(1, static_cast<int>(std::ceil(cx + ex)) - left), fh = std::max(1, static_cast<int>(std::ceil(cy + ey)) - top);
   vector<uint8_t> frame;
   B2SProDynamicLighting::SnippetFrameCrop(bulb, image, left, top, fw, fh, frame);
   for (int y = 0; y < fh; y++)
      for (int x = 0; x < fw; x++)
      {
         const int mx = left + x, my = top + y;
         const int alpha = (mx >= 0 && my >= 0 && mx < mask.w && my < mask.h) ? mask.alpha[static_cast<size_t>(my) * mask.w + mx] : 0;
         uint8_t& pa = frame[(static_cast<size_t>(y) * fw + x) * 4 + 3];
         pa = static_cast<uint8_t>((pa * alpha + 127) / 255);
      }
   UpdateTexture(&mask.frame, fw, fh, VPXTEXFMT_sRGBA8, frame.data());
   ctx->DrawImage(ctx, mask.frame, 1.f, 1.f, 1.f, bulb.m_brightness, 0.f, 0.f, static_cast<float>(fw), static_cast<float>(fh), 0.f, 0.f, 0.f, ContentX(static_cast<float>(left)),
      ContentY(static_cast<float>(top)), static_cast<float>(fw), static_cast<float>(fh));
   return true;
}

void B2SRenderer::DrawProImage(VPXRenderContext2D* ctx, const B2SBulb& bulb, VPXTexture texture)
{
   if (texture == nullptr)
      return;
   if (bulb.m_pro && bulb.m_pro->physicsBall && !bulb.m_pro->physicsSelectionMaskPng.empty() && bulb.m_pro->rt.motion.active && DrawProMaskedBall(ctx, bulb, texture))
      return;
   const VPXTextureInfo* const tex = GetTextureInfo(texture);
   // Native rotation and pivots turn around their pivot point; rolling motion pictures around their center
   const bool nativePivot = bulb.m_pro && bulb.m_pro->rt.driven && bulb.m_pro->rt.nativeRotation;
   const float pivotU = nativePivot ? bulb.m_pro->pivotX : 0.5f;
   const float pivotV = nativePivot ? bulb.m_pro->pivotY : 0.5f;
   // A moving picture is drawn at its current rectangle
   float x = static_cast<float>(bulb.m_locationX), y = static_cast<float>(bulb.m_locationY), w = static_cast<float>(bulb.m_width), h = static_cast<float>(bulb.m_height);
   if (bulb.m_pro && bulb.m_pro->rt.motion.active)
   {
      const ProMotion& m = bulb.m_pro->rt.motion;
      x = m.x;
      y = m.y;
      w = m.w;
      h = m.h;
   }
   ctx->DrawImage(ctx, texture, 1.f, 1.f, 1.f, bulb.m_brightness, 0.f, 0.f, static_cast<float>(tex->width), static_cast<float>(tex->height), static_cast<float>(tex->width) * pivotU,
      static_cast<float>(tex->height) * pivotV, bulb.m_drawRotation, ContentX(x), ContentY(y), w, h);
}

void B2SRenderer::DrawBulbImage(VPXRenderContext2D* ctx, const B2SBulb& bulbRef)
{
   {
      const B2SBulb* const bulb = &bulbRef;
      const float rotation = bulb->m_drawRotation;
      // B2S Pro: native rotation and pivots turn around their authored pivot point, in authored
      // coordinates (the backglass scaling is applied afterwards, as the reference's
      // NativeRotationDestinationPoints does)
      const bool proPivot = bulb->m_pro && bulb->m_pro->rt.driven;
      const float pivotU = proPivot ? bulb->m_pro->pivotX : 0.5f;
      const float pivotV = proPivot ? bulb->m_pro->pivotY : 0.5f;
      if (bulb->m_offImage && bulb->m_brightness < 1.f)
      {
         const VPXTextureInfo* const bulbTex = GetTextureInfo(bulb->m_offImage);
         ctx->DrawImage(ctx, bulb->m_offImage, bulb->m_lightColor.x, bulb->m_lightColor.y, bulb->m_lightColor.z, 1.f,
            0.f, 0.f, static_cast<float>(bulbTex->width), static_cast<float>(bulbTex->height),
            static_cast<float>(bulbTex->width) * pivotU, static_cast<float>(bulbTex->height) * pivotV, rotation,
            ContentX(static_cast<float>(bulb->m_locationX)), ContentY(static_cast<float>(bulb->m_locationY)), static_cast<float>(bulb->m_width), static_cast<float>(bulb->m_height));
      }
      if (bulb->m_image)
      {
         const VPXTextureInfo* const bulbTex = GetTextureInfo(bulb->m_image);
         ctx->DrawImage(ctx, bulb->m_image, bulb->m_lightColor.x, bulb->m_lightColor.y, bulb->m_lightColor.z, bulb->m_brightness,
            0.f, 0.f, static_cast<float>(bulbTex->width), static_cast<float>(bulbTex->height),
            static_cast<float>(bulbTex->width) * pivotU, static_cast<float>(bulbTex->height) * pivotV, rotation,
            ContentX(static_cast<float>(bulb->m_locationX)), ContentY(static_cast<float>(bulb->m_locationY)), static_cast<float>(bulb->m_width), static_cast<float>(bulb->m_height));
      }
   }
}

void B2SRenderer::RenderScores(VPXRenderContext2D* ctx, B2SServer* server, const B2SScores& scores, float elapsed, const std::function<void(const B2SScore&)>* beforeScore)
{
   if (server == nullptr)
      return;
   // B2SHideScoreDisplays/animations only hide LED score displays, reel displays keep rendering
   const bool ledDisplaysHidden = server->AreScoreDisplaysHidden();

   vector<SegElementType> segTypes;
   vector<float> luminances;
   vector<VPXSegDisplayRenderStyle> styles;
   vector<VPXSegDisplayHint> hints;
   m_segSources.With([&segTypes, &styles, &hints, &luminances, server](const std::vector<SegSrcId>& segDisplays){
      int digitIndex = 1;
      for (const auto& display : segDisplays)
      {
         SegDisplayFrame state = display.GetState(display.callContext);
         for (unsigned int i = 0; i < display.nElements; i++)
         {
            VPXSegDisplayHint hint = VPXSegDisplayHint::Generic;
            VPXSegDisplayRenderStyle style = VPXSegDisplayRenderStyle::VPXSegStyle_Plasma;
            if ((display.hardware & CTLPI_SEG_HARDWARE_FAMILY_MASK) == CTLPI_SEG_HARDWARE_NEON_PLASMA)
               style = VPXSegDisplayRenderStyle::VPXSegStyle_Plasma;
            else if ((display.hardware & CTLPI_SEG_HARDWARE_FAMILY_MASK) == CTLPI_SEG_HARDWARE_VFD_GREEN)
               style = VPXSegDisplayRenderStyle::VPXSegStyle_GreenVFD;
            else if ((display.hardware & CTLPI_SEG_HARDWARE_FAMILY_MASK) == CTLPI_SEG_HARDWARE_VFD_BLUE)
            {
               style = VPXSegDisplayRenderStyle::VPXSegStyle_BlueVFD;
               if (display.hardware == CTLPI_SEG_HARDWARE_GTS1_4DIGIT //
                  || display.hardware == CTLPI_SEG_HARDWARE_GTS1_6DIGIT //
                  || display.hardware == CTLPI_SEG_HARDWARE_GTS80A_7DIGIT //
                  || display.hardware == CTLPI_SEG_HARDWARE_GTS80B_20DIGIT //
               )
                  hint = VPXSegDisplayHint::Gottlieb;
            }
            segTypes.push_back(display.elementType[i]);
            styles.push_back(style);
            hints.push_back(hint);

            int bitState = 0;
            for (int j = 0; j < 16; j++)
            {
               luminances.push_back(state.frame[i * 16 + j]);
               if (state.frame[i * 16 + j] > 0.5f)
                  bitState |= 1 << j;
            }
            int ret;
            switch (bitState & ~0x80) // Remove the comma
            {
            // 7-segment stuff
            case 0x003F: ret = 0; break;
            case 0x0006: ret = 1; break;
            case 0x005B: ret = 2; break;
            case 0x004F: ret = 3; break;
            case 0x0066: ret = 4; break;
            case 0x006D: ret = 5; break;
            case 0x007D: ret = 6; break;
            case 0x0007: ret = 7; break;
            case 0x007F: ret = 8; break;
            case 0x006F: ret = 9; break;
            // Additional 10-segment stuff
            case 0x0300: ret = 1; break;
            case 0x007C: ret = 6; break;
            case 0x0067: ret = 9; break;
            // Default is empty
            default: ret = -1; break;
            }
            server->B2SSetScoreDigit(digitIndex, ret);
            // Keep the raw segment mask: script digits decode to value-only, but frame-fed displays
            // need the full mask (alphanumeric glyphs do not decode to a digit). Must be applied
            // after B2SSetScoreDigit which clears the mask.
            server->ApplyScoreSegments(digitIndex, bitState);
            digitIndex++;
         }
      }
   });

   // Build the per-player score strings: right aligned, space padded on the left, then split
   // across the player displays in file order (each display consuming its digit count)
   ankerl::unordered_dense::map<int, string> playerScoreText;
   ankerl::unordered_dense::map<int, size_t> playerScoreOffset;
   for (const auto& score : scores.m_scores)
      if (score.m_b2sPlayerNo > 0)
         playerScoreText[score.m_b2sPlayerNo].append(static_cast<size_t>(std::max(0, score.m_digits)), ' ');
   for (auto& [player, text] : playerScoreText)
   {
      const string scoreText = std::to_string(abs(server->GetPlayerScore(player)));
      if (scoreText.length() >= text.length())
         text = scoreText.substr(scoreText.length() - text.length());
      else
         text.replace(text.length() - scoreText.length(), scoreText.length(), scoreText);
   }

   // B2S Pro: with a layer callback the displays are drawn in rank order (the callback draws the
   // layers below each display first). Player digits are still split in file order: each display
   // starts at the offset the file-order pass would have reached.
   vector<const B2SScore*> drawOrder;
   std::unordered_map<const B2SScore*, size_t> proPlayerOffset;
   for (const auto& reel : scores.m_scores)
      drawOrder.push_back(&reel);
   if (beforeScore != nullptr)
   {
      ankerl::unordered_dense::map<int, size_t> running;
      for (const auto& reel : scores.m_scores)
      {
         if (reel.IsHidden() || (ledDisplaysHidden && (reel.m_scoreType == B2SScoreRenderer::Dream7 || reel.m_scoreType == B2SScoreRenderer::RenderedLED)))
            continue;
         if (reel.m_b2sPlayerNo != 0)
         {
            proPlayerOffset[&reel] = running[reel.m_b2sPlayerNo];
            running[reel.m_b2sPlayerNo] += static_cast<size_t>(std::max(0, reel.m_digits));
         }
      }
      std::stable_sort(drawOrder.begin(), drawOrder.end(), [](const B2SScore* a, const B2SScore* b) { return a->m_zOrder < b->m_zOrder; });
   }

   for (const B2SScore* const reelPtr : drawOrder)
   {
      const B2SScore& reel = *reelPtr;
      if (beforeScore != nullptr)
      {
         (*beforeScore)(reel);
         if (const auto it = proPlayerOffset.find(&reel); it != proPlayerOffset.end())
            playerScoreOffset[reel.m_b2sPlayerNo] = it->second;
      }

      // Skip hidden displays (DisplayState=1), and LED displays when score displays are hidden
      if (reel.IsHidden() || (ledDisplaysHidden && (reel.m_scoreType == B2SScoreRenderer::Dream7 || reel.m_scoreType == B2SScoreRenderer::RenderedLED)))
         continue;

      // Nothing to draw, and the digit width below would divide by zero
      if (reel.m_digits == 0)
         continue;

      // Skip digits located on the grill when the grill is hidden
      if (static_cast<float>(reel.m_locY) > ctx->srcHeight)
      {
         if (reel.m_b2sPlayerNo != 0)
            playerScoreOffset[reel.m_b2sPlayerNo] += static_cast<size_t>(std::max(0, reel.m_digits));
         continue;
      }

      // B2S Pro: a rotated or perspective LED display is rasterized like the reference (Dream7Display /
      // B2SLEDBox render flat content that ScoreTransform.DrawWarped maps onto the quad)
      if (m_b2s->m_unifiedLayerDeck && reel.m_hasTransform && (reel.m_scoreType == B2SScoreRenderer::Dream7 || reel.m_scoreType == B2SScoreRenderer::RenderedLED))
      {
         const int usedLEDType = server->GetUsedLEDType();
         const bool dream7 = (usedLEDType == 2) || (usedLEDType != 1 && reel.m_scoreType == B2SScoreRenderer::Dream7);
         const ProScoreTransform transform = ProTransformOf(reel);
         RenderLegacyScoreDisplay(ctx, server, reel, dream7, segTypes, playerScoreText, playerScoreOffset, &transform);
         continue;
      }

      // Legacy segment rendering (opt-out of the upgraded renderer): rasterize like the original
      // B2S server — one Dream7Display per LED display, or one simple LED box per digit
      if (!upgradedSegRenderingProp_Get() && (reel.m_scoreType == B2SScoreRenderer::Dream7 || reel.m_scoreType == B2SScoreRenderer::RenderedLED))
      {
         const int usedLEDType = server->GetUsedLEDType();
         const bool dream7 = (usedLEDType == 2) || (usedLEDType != 1 && reel.m_scoreType == B2SScoreRenderer::Dream7);
         RenderLegacyScoreDisplay(ctx, server, reel, dream7, segTypes, playerScoreText, playerScoreOffset);
         continue;
      }

      const float width = (static_cast<float>(reel.m_width) - 0.5f * static_cast<float>((reel.m_digits - 1) * reel.m_spacing)) / static_cast<float>(reel.m_digits);

      // Reel rolling: resolve the rolling direction, the illuminated image selection and the per-digit
      // hold flags. A digit waits while a less significant digit has a pending 9->0 / 0->9 wrap (carry).
      const bool rollUp = scores.m_reelRollingDirection != B2SReelRollingDirection::Down;
      // ReelIlluB2SID is in lamp-id space: script B2SSetData writes and ROM lamp states share the
      // channel; the most recent change wins
      {
         const float prevIllu = reel.m_reelIllu;
         reel.m_reelIlluUpdater();
         if (server != nullptr && prevIllu >= 0.f && reel.m_reelIllu != prevIllu)
            reel.m_reelIlluStamp = server->NextStamp();
      }
      float illuState = std::max(0.f, reel.m_reelIllu);
      uint64_t illuLampStamp = 0;
      float scriptedIllu = 0.f;
      if (server != nullptr && server->GetScriptedLampState(reel.m_reelIlluB2SID, scriptedIllu, &illuLampStamp) && illuLampStamp > reel.m_reelIlluStamp)
         illuState = scriptedIllu;
      const bool illuminated = reel.m_reelIlluB2SID > 0 && (reel.m_reelIlluB2SValue > 0 ? (static_cast<int>(illuState) == reel.m_reelIlluB2SValue) : (illuState != 0.f));
      vector<char> hold(static_cast<size_t>(std::max(0, reel.m_digits)), 0);
      bool lowerWrapPending = false;
      for (int i = reel.m_digits - 1; i >= 0; i--)
      {
         hold[static_cast<size_t>(i)] = lowerWrapPending ? 1 : 0;
         if (const auto it = m_reelDigits.find(reel.m_resolvedStartDigit + i); it != m_reelDigits.end() && it->second.HasPendingWrap(rollUp))
            lowerWrapPending = true;
      }

      for (int i = 0; i < reel.m_digits; i++)
      {
         const float x = static_cast<float>(reel.m_locX) + static_cast<float>(i) * (width + 0.5f * static_cast<float>(reel.m_spacing));
         int digit = 0;
         const int index = reel.m_resolvedStartDigit + i;
         if (reel.m_b2sPlayerNo != 0)
         {
            const string& text = playerScoreText[reel.m_b2sPlayerNo];
            const size_t offset = playerScoreOffset[reel.m_b2sPlayerNo] + static_cast<size_t>(i);
            const char c = offset < text.length() ? text[offset] : ' ';
            const int raw = (c >= '0' && c <= '9') ? c - '0' : -1;
            digit = (reel.m_scoreType == B2SScoreRenderer::Dream7 || reel.m_scoreType == B2SScoreRenderer::RenderedLED) ? raw : (raw < 0 ? 0 : raw);
            if (i == reel.m_digits - 1)
               playerScoreOffset[reel.m_b2sPlayerNo] += static_cast<size_t>(reel.m_digits);
         }
         else
            digit = server->GetScoreDigit(index);

         switch (reel.m_scoreType)
         {
         case B2SScoreRenderer::LED:
            // Black Pyramid (Bally 1984)
         case B2SScoreRenderer::ImportedLED:
            // Apache! (Taito 1978)
            // LED & ImportedLED seem to be the same as reel but without animation
         case B2SScoreRenderer::Reel:
            // Volkan Steel and Metal (Original 2023), Hang Glider (Bally 1976) => Simple reels, no animations
            // ? => Simple reels with animations & sounds
            {
               int shownDigit = digit;
               int intermediate = 0;
               B2SReelDigit& st = m_reelDigits[index];
               const B2SReelImage* cur = m_b2s->m_reels.GetImage(reel.m_reelType, st.Current(), illuminated, reel.m_reelIlluImageSet);
               const int intermediates = cur ? cur->m_countOfIntermediate : 0;
               st.SetIlluminated(illuminated, intermediates);
               // Player displays are updated from decoded pinmame frames so they always roll;
               // script driven displays roll when B2SSetScore/B2SSetReel raised the roll flag.
               // LED image displays never roll (B2SReelBox::isLED).
               st.SetTarget(digit, reel.m_scoreType == B2SScoreRenderer::Reel && (reel.m_b2sPlayerNo != 0 || server->ConsumeScoreDigitRoll(index)));
               // B2S Pro Realistic reel style: the drum turns smoothly instead of showing intermediate images
               const bool smoothRoll = m_b2s->m_unifiedLayerDeck && reel.m_reel3DEnabled && reel.m_reel3DStyle == 1 && reel.m_scoreType == B2SScoreRenderer::Reel;
               if (!hold[static_cast<size_t>(i)]
                  && (smoothRoll ? st.UpdateSmooth(elapsed, ProReelStepMs(scores.m_reelRollingInterval, intermediates), rollUp)
                                 : st.Update(elapsed, scores.m_reelRollingInterval, intermediates, rollUp)))
               {
                  // Reel roll sound: per-digit Sound1..N attribute, or the display wide Sound attribute
                  const string& soundName
                     = (i < static_cast<int>(reel.m_soundNames.size()) && !reel.m_soundNames[static_cast<size_t>(i)].empty()) ? reel.m_soundNames[static_cast<size_t>(i)] : reel.m_soundName;
                  if (!soundName.empty() && soundName != "stille"sv)
                     server->B2SPlaySound(soundName);
               }
               shownDigit = st.Current();
               intermediate = smoothRoll ? 0 : st.Intermediate();
               const B2SReelImage* reelImage = m_b2s->m_reels.GetImage(reel.m_reelType, shownDigit, illuminated, reel.m_reelIlluImageSet);
               const VPXTexture texture = (reelImage && intermediate > 0 && intermediate <= static_cast<int>(reelImage->m_intermediates.size()))
                  ? reelImage->m_intermediates[static_cast<size_t>(intermediate - 1)]
                  : nullptr;
               if (reelImage && (texture != nullptr || reelImage->m_image != nullptr))
               {
                  const VPXTexture img = texture != nullptr ? texture : reelImage->m_image;
                  if (m_b2s->m_unifiedLayerDeck && (reel.m_keepDigitAspect || reel.m_reel3DEnabled || reel.m_hasTransform))
                  {
                     // B2S Pro Realistic reel style: the strip around the shown number (offsets -2..+2) of the same image set
                     VPXTexture strip[5] = { nullptr, nullptr, nullptr, nullptr, nullptr };
                     if (reel.m_reel3DEnabled && reel.m_reel3DStyle == 1 && shownDigit >= 0 && shownDigit <= 9)
                     {
                        for (int offset = -2; offset <= 2; offset++)
                        {
                           const B2SReelImage* n = offset == 0 ? nullptr : m_b2s->m_reels.GetImage(reel.m_reelType, (shownDigit + offset + 10) % 10, illuminated, reel.m_reelIlluImageSet);
                           strip[offset + 2] = n ? n->m_image : nullptr;
                        }
                     }
                     DrawProReelDigit(ctx, reel, i, img, illuminated, x, width, strip, smoothRoll ? st.Roll() : 0.0); // B2S Pro
                  }
                  else
                  {
                     const VPXTextureInfo* texInfo = GetTextureInfo(img);
                     ctx->DrawImage(ctx, img, 1.f, 1.f, 1.f, 1.f, //
                        0.f, 0.f, static_cast<float>(texInfo->width), static_cast<float>(texInfo->height), //
                        0.f, 0.f, 0.f, // No rotation
                        ContentX(x), ContentY(static_cast<float>(reel.m_locY)), width, static_cast<float>(reel.m_height));
                  }
               }
            }
            break;

         case B2SScoreRenderer::Dream7:
            // Asteroid Annie (Gottlieb 1980)
         case B2SScoreRenderer::RenderedLED:
         {
            std::array<float, 16> brightness;
            // Segment layout declared by the ReelType suffix (Dream7LEDx/RenderedLEDx)
            SegElementType segType = reel.m_ledSegments == 7 ? SegElementType::CTLPI_SEG_LAYOUT_7
               : reel.m_ledSegments == 10                    ? SegElementType::CTLPI_SEG_LAYOUT_9
               : reel.m_ledSegments == 14                    ? SegElementType::CTLPI_SEG_LAYOUT_14
                                                             : SegElementType::CTLPI_SEG_LAYOUT_14;
            VPXSegDisplayRenderStyle style = VPXSegDisplayRenderStyle::VPXSegStyle_Plasma;
            VPXSegDisplayHint hint = VPXSegDisplayHint::Generic;
            if (index > 0 && index * 16 <= (int)luminances.size())
            {
               segType = segTypes[index - 1];
               style = styles[index - 1];
               hint = hints[index - 1];
               memcpy(brightness.data(), luminances.data() + (index - 1) * 16, 16 * sizeof(float));
            }
            else
            {
               // Script driven digit: an explicit B2SSetLED segment mask takes precedence,
               // otherwise the value is mapped to its standard 7 segment pattern
               const int segMask = server->GetScoreDigitSegments(index);
               static constexpr uint16_t digitSegments[10] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };
               const uint16_t bits = segMask >= 0 ? static_cast<uint16_t>(segMask) : (digit >= 0 && digit < 10) ? digitSegments[digit] : 0;
               for (int j = 0; j < 16; j++)
                  brightness[j] = ((bits >> j) & 1) ? 1.f : 0.f;
               if (reel.m_scoreType == B2SScoreRenderer::RenderedLED)
                  style = VPXSegDisplayRenderStyle::VPXSegStyle_GenLED;
            }
            // UsedLEDType plugin setting forces Dream7 (2) or rendered LEDs (1) whatever the table declares
            const int usedLEDType = server->GetUsedLEDType();
            if (usedLEDType == 1)
               style = VPXSegDisplayRenderStyle::VPXSegStyle_GenLED;
            else if (usedLEDType == 2)
               style = VPXSegDisplayRenderStyle::VPXSegStyle_Plasma;
            // The styled profiles embed their own display color so the emitter tint must stay neutral
            // for them (it is multiplied over the profile color, tinting twice); only the generic
            // styles take the authored lit color. Same for the glass: the B2S dark color is the unlit
            // segment color, not a glass tint, and multiplied over the output it darkens the display.
            const vec4 litColor = (style == VPXSegDisplayRenderStyle::VPXSegStyle_GenPlasma || style == VPXSegDisplayRenderStyle::VPXSegStyle_GenLED)
               ? vec4(InvsRGB(reel.m_reelLitColor.x), InvsRGB(reel.m_reelLitColor.y), InvsRGB(reel.m_reelLitColor.z), 1.f)
               : vec4(1.f, 1.f, 1.f, 1.f);
            ctx->DrawSegDisplay(ctx, style, hint,
               // First layer: neutral glass, there is no glass texture
               nullptr, 1.f, 1.f, 1.f, 0.15f, // Glass texture, tint and roughness
               0.f, 0.f, 0.f, 0.f, // Glass texture coordinates (inside overall glass texture, cut for each element)
               0.f, 0.f, 0.f, // Glass lighting from room
               // Second layer: emitter tinted with the lit segment color
               segType, brightness.data(), // Segment emitter type and brightness array
               litColor.x, litColor.y, litColor.z, 1.f, 1.f, // Emitter tint, emitter brightness, emitter alpha
               0.f, 0.f, 0.f, 0.f, // Emitter padding (from glass border)
               // Render quad
               ContentX(x), ctx->srcHeight - m_layoutOY - static_cast<float>(reel.m_locY + reel.m_height), width, static_cast<float>(reel.m_height));
         }
            break;
         }
      }

      // B2S Pro: the strips between the digits of a 3D reel display get their part of the window overlay, as
      // the Designer draws it across the whole score (formBackglass ReelGapOverlay), at this score's place
      if (m_b2s->m_unifiedLayerDeck && reel.m_reel3DEnabled && reel.m_digits > 1
         && (reel.m_scoreType == B2SScoreRenderer::Reel || reel.m_scoreType == B2SScoreRenderer::LED || reel.m_scoreType == B2SScoreRenderer::ImportedLED))
         DrawProReelGaps(ctx, reel, width);
   }
}


// Legacy segment rendering (UpgradedSegmentRendering opt-out), matching the original B2S server
// implementation: a Dream7Display spans the whole score display, rendered LED boxes are one box
// per digit. Both rasterize the segments into a texture which is redrawn only on changes
// Segment count of a CTLPI_SEG_LAYOUT_xxx element type, matching B2SSegmentUntranslateBitCode's 'ledSegments' values
static int SegLayoutSegments(SegElementType type)
{
   switch (type)
   {
   case CTLPI_SEG_LAYOUT_7:
   case CTLPI_SEG_LAYOUT_7C:
   case CTLPI_SEG_LAYOUT_7D: return 7;
   case CTLPI_SEG_LAYOUT_9:
   case CTLPI_SEG_LAYOUT_9C: return 10;
   case CTLPI_SEG_LAYOUT_16: return 16;
   default: return 14;
   }
}

void B2SRenderer::RenderLegacyScoreDisplay(VPXRenderContext2D* ctx, B2SServer* server, const B2SScore& reel, bool dream7, const std::vector<SegElementType>& segTypes,
   const ankerl::unordered_dense::map<int, string>& playerScoreText, ankerl::unordered_dense::map<int, size_t>& playerScoreOffset, const ProScoreTransform* transform)
{
   // B2S Pro: the display quad (formBackglass: DisplayQuad(LocX, LocY, Width, Height)) and the digit
   // slices of a rendered LED display (same layout as the reels)
   ProQuad displayQuad {};
   const int proDigitWidth = static_cast<int>(std::nearbyint((reel.m_width - (reel.m_digits - 1) * reel.m_spacing / 2.0) / std::max(1, reel.m_digits)));
   const auto digitQuad = [&](int i)
   {
      const float u0 = (1.f + static_cast<float>(i) * (static_cast<float>(proDigitWidth) + static_cast<float>(reel.m_spacing) / 2.f)) / static_cast<float>(reel.m_width);
      const float u1 = std::min(1.f, u0 + static_cast<float>(proDigitWidth) / static_cast<float>(reel.m_width));
      return ProSubQuad(displayQuad, u0, u1, 1.f / static_cast<float>(reel.m_height), 1.f);
   };
   if (transform != nullptr)
      displayQuad = transform->DisplayQuad(static_cast<float>(reel.m_locX), static_cast<float>(reel.m_locY), static_cast<float>(reel.m_width), static_cast<float>(reel.m_height));
   LegacySegDisplay& state = m_legacySegDisplays[&reel];
   if (!state.built || state.dream7 != dream7)
   {
      // (Re)build the display, also on a UsedLEDType mode change
      for (VPXTexture tex : state.warped) // B2S Pro warped textures of the previous build
         if (tex != nullptr)
            DeleteTexture(tex);
      state = LegacySegDisplay {};
      state.dream7 = dream7;
      const uint8_t litR = static_cast<uint8_t>(std::clamp(reel.m_reelLitColor.x * 255.f + 0.5f, 0.f, 255.f));
      const uint8_t litG = static_cast<uint8_t>(std::clamp(reel.m_reelLitColor.y * 255.f + 0.5f, 0.f, 255.f));
      const uint8_t litB = static_cast<uint8_t>(std::clamp(reel.m_reelLitColor.z * 255.f + 0.5f, 0.f, 255.f));
      const uint8_t darkR = static_cast<uint8_t>(std::clamp(reel.m_reelDarkColor.x * 255.f + 0.5f, 0.f, 255.f));
      const uint8_t darkG = static_cast<uint8_t>(std::clamp(reel.m_reelDarkColor.y * 255.f + 0.5f, 0.f, 255.f));
      const uint8_t darkB = static_cast<uint8_t>(std::clamp(reel.m_reelDarkColor.z * 255.f + 0.5f, 0.f, 255.f));
      const uint32_t litColor = seg::RGB(litR, litG, litB);
      const uint32_t darkColor = seg::RGB(darkR, darkG, darkB);
      if (dream7)
      {
         auto d7 = std::make_unique<seg::Dream7Display>(m_vpxApi);
         if (transform != nullptr)
         {
            // B2S Pro: flat content at the quad's size (ScoreTransformState.ContentSize)
            int cw, ch;
            ProContentSize(displayQuad, cw, ch);
            d7->SetRect({ 0, 0, cw, ch });
            state.quads.push_back(displayQuad);
         }
         else
            d7->SetRect({ reel.m_locX - 1, reel.m_locY, reel.m_width, reel.m_height });
         d7->SetType(reel.m_ledSegments == 10 ? seg::SegmentNumberType_TenSegment : reel.m_ledSegments == 14 ? seg::SegmentNumberType_FourteenSegment : seg::SegmentNumberType_SevenSegment);
         d7->SetScaleMode(seg::ScaleMode_Stretch);
         d7->SetDigits(reel.m_digits);
         d7->SetSpacing(static_cast<float>(reel.m_spacing) * 5.f);
         // Reference color wiring: the glass colors derive from the authored lit segment color
         d7->SetLightColor(litColor);
         d7->SetGlassColor(seg::RGB(static_cast<uint8_t>(std::min(255, litR + 50)), static_cast<uint8_t>(std::min(255, litG + 50)), static_cast<uint8_t>(std::min(255, litB + 50))));
         d7->SetGlassColorCenter(seg::RGB(static_cast<uint8_t>(std::min(255, litR + 70)), static_cast<uint8_t>(std::min(255, litG + 70)), static_cast<uint8_t>(std::min(255, litB + 70))));
         d7->SetOffColor(darkColor);
         d7->SetGlassAlpha(140);
         d7->SetGlassAlphaCenter(255);
         d7->SetThickness(static_cast<float>(reel.m_thickness) / 100.f * 1.2f);
         d7->SetShear(static_cast<float>(reel.m_shear) / 100.f);
         d7->SetGlow(static_cast<float>(std::lround(static_cast<float>(reel.m_glow) / 100.f)));
         // 'Taxi' patch to shear the third LED display (reference FormBackglass)
         if (reel.m_id == 3 && string_to_lower(m_b2s->m_name) == "taxi"s)
         {
            d7->SetAngle(4.f);
            d7->SetShear(d7->GetShear() / 2.f);
         }
         state.display = std::move(d7);
      }
      else
      {
         const int width = static_cast<int>(std::lround((reel.m_width - (reel.m_digits - 1) * reel.m_spacing / 2.f) / reel.m_digits));
         state.boxes.reserve(static_cast<size_t>(reel.m_digits));
         for (int i = 0; i < reel.m_digits; i++)
         {
            auto box = std::make_unique<seg::LEDBox>(m_vpxApi);
            const int x = static_cast<int>(std::lround((reel.m_locX - 1) + i * (width + reel.m_spacing / 2.f)));
            if (transform != nullptr)
            {
               const ProQuad quad = digitQuad(i);
               int cw, ch;
               ProContentSize(quad, cw, ch);
               box->SetRect({ 0, 0, cw, ch });
               state.quads.push_back(quad);
            }
            else
               box->SetRect({ x, reel.m_locY, width, reel.m_height });
            box->SetLEDType(reel.m_ledSegments == 10 ? seg::eLEDType_LED10 : reel.m_ledSegments == 14 ? seg::eLEDType_LED14 : seg::eLEDType_LED8);
            box->SetLitLEDSegmentColor(litColor);
            box->SetDarkLEDSegmentColor(darkColor);
            state.boxes.push_back(std::move(box));
         }
      }
      state.built = true;
   }

   // Feed the digits: an explicit script segment mask (B2SSetLED/B2SSetLEDDisplay, untranslated
   // back to the Dream7 bit order these renderers expect) wins, else the digit glyph
   for (int i = 0; i < reel.m_digits; i++)
   {
      const int index = reel.m_resolvedStartDigit + i;
      string text;
      if (reel.m_b2sPlayerNo != 0)
      {
         const auto it = playerScoreText.find(reel.m_b2sPlayerNo);
         const size_t offset = playerScoreOffset[reel.m_b2sPlayerNo] + static_cast<size_t>(i);
         text = string(1, (it != playerScoreText.end() && offset < it->second.length()) ? it->second[offset] : ' ');
      }
      else
      {
         // TODO As the original B2SSetScoreDigit: decimal text, of which only the first character is drawn (10-15 show '1').
         // TODO Unlike the original, a digit the script never set shows '0' (the original left it blank)
         text = std::to_string(server->GetScoreDigit(index));
      }
      const int segMask = server->GetScoreDigitSegments(index);
      // Source segment layout for the mask untranslation: the PinMAME frame element's own layout
      // when the digit is frame-fed (it may differ from the display's declared type, e.g. a Williams
      // 16 segment alphanum source driving a Dream7LED14 display), else the declared segment count.
      const int srcSegs = (index > 0 && index <= static_cast<int>(segTypes.size())) ? SegLayoutSegments(segTypes[static_cast<size_t>(index - 1)]) : reel.m_ledSegments;
      if (dream7)
      {
         if (segMask >= 0)
            state.display->SetValue(i, B2SSegmentUntranslateBitCode(static_cast<uint32_t>(segMask), srcSegs));
         else
            state.display->SetValue(i, text);
      }
      else if (i < static_cast<int>(state.boxes.size()))
      {
         if (segMask >= 0)
            state.boxes[static_cast<size_t>(i)]->SetValue(B2SSegmentUntranslateBitCode(static_cast<uint32_t>(segMask), srcSegs));
         else
            state.boxes[static_cast<size_t>(i)]->SetText(text);
      }
   }
   if (reel.m_b2sPlayerNo != 0)
      playerScoreOffset[reel.m_b2sPlayerNo] += static_cast<size_t>(reel.m_digits);

   if (transform != nullptr)
   {
      // B2S Pro: rasterize flat, warp onto the quad when the pixels changed, draw the warped texture
      const size_t count = dream7 ? 1 : state.boxes.size();
      state.warped.resize(count, nullptr);
      state.warpX.resize(count, 0);
      state.warpY.resize(count, 0);
      state.warpW.resize(count, 0);
      state.warpH.resize(count, 0);
      for (size_t k = 0; k < count && k < state.quads.size(); k++)
      {
         bool changed;
         const seg::SegGraphics* graphics;
         if (dream7)
         {
            changed = state.display->Rasterize();
            graphics = state.display->GetGraphics();
         }
         else
         {
            changed = state.boxes[k]->Rasterize();
            graphics = state.boxes[k]->GetGraphics();
         }
         if (graphics == nullptr)
            continue;
         if (changed || state.warped[k] == nullptr)
         {
            ProImage flat;
            flat.Allocate(graphics->GetWidth(), graphics->GetHeight());
            memcpy(flat.px.data(), graphics->GetPixels().data(), flat.px.size());
            ProImage out;
            int bx, by;
            ProWarp(flat, state.quads[k], bx, by, out);
            UpdateTexture(&state.warped[k], out.w, out.h, VPXTEXFMT_sRGBA8, out.px.data());
            state.warpX[k] = bx;
            state.warpY[k] = by;
            state.warpW[k] = out.w;
            state.warpH[k] = out.h;
         }
         if (state.warped[k] != nullptr)
            ctx->DrawImage(ctx, state.warped[k], 1.f, 1.f, 1.f, 1.f, 0.f, 0.f, static_cast<float>(state.warpW[k]), static_cast<float>(state.warpH[k]), 0.f, 0.f, 0.f,
               ContentX(static_cast<float>(state.warpX[k])), ContentY(static_cast<float>(state.warpY[k])), static_cast<float>(state.warpW[k]), static_cast<float>(state.warpH[k]));
      }
      return;
   }
   if (dream7)
   {
      state.display->SetViewOffset(m_layoutOX, m_layoutOY); // Placed with the KeepAspectRatio placement
      state.display->Render(ctx);
   }
   else
      for (auto& box : state.boxes)
      {
         box->SetViewOffset(m_layoutOX, m_layoutOY);
         box->Render(ctx);
      }
}

ProScoreTransform B2SRenderer::ProTransformOf(const B2SScore& score)
{
   ProScoreTransform transform;
   transform.rotationAngle = score.m_rotationAngle;
   transform.perspectiveDepth = score.m_perspectiveDepth;
   transform.perspectiveLeftScale = score.m_perspectiveLeftScale;
   transform.perspectiveRightScale = score.m_perspectiveRightScale;
   transform.active = score.m_hasTransform;
   return transform;
}

// B2S Pro: the display wide 3D window overlay (Reel3DEffect.CreateWindowOverlay), with the reel dividers
// centred on the digit boundaries as the digits below are laid out (xi - locX, digit width designW)
static ProImage ProReelDisplayOverlay(const B2SScore& reel, int designW, int designH)
{
   std::vector<double> centers;
   if (reel.m_reel3DDividerWidth > 0 && reel.m_digits > 1)
   {
      const int locX = reel.m_locX - 1;
      for (int i = 0; i < reel.m_digits - 1; i++)
      {
         const int left = static_cast<int>(std::nearbyint(locX + i * (designW + reel.m_spacing / 2.0))) - locX;
         const int next = static_cast<int>(std::nearbyint(locX + (i + 1) * (designW + reel.m_spacing / 2.0))) - locX;
         centers.push_back((left + designW + next) / 2.0);
      }
   }
   return ProReel3DWindowOverlay(reel.m_width, designH, reel.m_reel3DDepth, reel.m_reel3DGlass, reel.m_reel3DGlassStyle,
      centers.empty() ? nullptr : &centers, reel.m_reel3DDividerWidth);
}

// B2S Pro: B2SReelBox.OnPaint / DrawReelImage / DrawReelImageCore. The digit image is fitted to the
// authored digit size (KeepDigitAspect, ReelDigitWidth), given the 3D reel look (Reel3D*), covered
// by its part of the display wide 3D window overlay, then mapped onto its slice of the rotated or
// perspective score (ScoreTransform). Rebuilt only when the shown image or the backlight changes.
void B2SRenderer::DrawProReelDigit(VPXRenderContext2D* ctx, const B2SScore& reel, int digit, VPXTexture image, bool illuminated, float x, float width, const VPXTexture* strip, double roll)
{
   const int digits = std::max(1, reel.m_digits);
   const int designW = std::max(1, static_cast<int>(std::nearbyint((reel.m_width - (digits - 1) * reel.m_spacing / 2.0) / digits)));
   const int designH = std::max(1, reel.m_height);
   // A triggered 3D reel keeps its depth and glass while unlit; its backlight follows the reel light
   const bool backlight = reel.m_reelIlluB2SID <= 0 || illuminated;
   ProDigitCache& cache = m_proDigits[{ &reel, digit }];
   if (!cache.valid || cache.source != image || cache.backlight != backlight || cache.roll != roll)
   {
      cache.valid = true;
      cache.source = image;
      cache.backlight = backlight;
      cache.roll = roll;
      auto decode = [this](VPXTexture tex) -> const ProImage*
      {
         if (tex == nullptr)
            return nullptr;
         auto it = m_proReelImages.find(tex);
         if (it == m_proReelImages.end())
         {
            ProImage decoded;
            ProReadTexture(tex, decoded);
            it = m_proReelImages.emplace(tex, std::move(decoded)).first;
         }
         return &it->second;
      };
      ProImage content = *decode(image);
      if (!content.Empty() && reel.m_reel3DEnabled && reel.m_reel3DStyle == 1)
      {
         // Realistic style: sized from the original artwork, on its drum (ReelRealistic)
         const ProImage* numbers[5];
         for (int n = 0; n < 5; n++)
            numbers[n] = n == 2 ? &content : decode(strip ? strip[n] : nullptr);
         content = ProReelRealisticRender(numbers, designW, designH, reel.m_keepDigitAspect, reel.m_keepDigitAspect ? reel.m_reelDigitWidth : 0,
            backlight ? reel.m_reel3DBrightness : 0, backlight ? reel.m_reel3DTemperature : 4000, reel.m_reel3DDepth, reel.m_reel3DCurve, reel.m_reel3DShadow, roll);
      }
      else
      {
         if (!content.Empty() && reel.m_keepDigitAspect)
            content = reel.m_reelDigitWidth > 0 ? ProReelStretchToWidth(content, designW, designH, reel.m_reelDigitWidth) : ProReelFit(content, designW, designH);
         if (!content.Empty() && reel.m_reel3DEnabled)
            content = ProReel3DRender(content, designW, designH, backlight ? reel.m_reel3DBrightness : 0, backlight ? reel.m_reel3DTemperature : 4000, reel.m_reel3DDepth);
      }

      ProQuad quad {};
      int targetW = designW, targetH = designH;
      if (reel.m_hasTransform)
      {
         const ProQuad displayQuad = ProTransformOf(reel).DisplayQuad(static_cast<float>(reel.m_locX), static_cast<float>(reel.m_locY), static_cast<float>(reel.m_width), static_cast<float>(reel.m_height));
         const float u0 = (1.f + static_cast<float>(digit) * (static_cast<float>(designW) + static_cast<float>(reel.m_spacing) / 2.f)) / static_cast<float>(reel.m_width);
         const float u1 = std::min(1.f, u0 + static_cast<float>(designW) / static_cast<float>(reel.m_width));
         quad = ProSubQuad(displayQuad, u0, u1, 1.f / static_cast<float>(reel.m_height), 1.f);
         ProContentSize(quad, targetW, targetH);
      }
      ProImage flat;
      flat.Allocate(targetW, targetH);
      if (!content.Empty())
         ProDrawScaled(flat, content, 0.0, 0.0, targetW, targetH, false);
      // Each digit crops its aligned piece of one display wide overlay
      if (reel.m_reel3DEnabled && reel.m_width > 2)
      {
         auto ov = m_proReelOverlays.find(&reel);
         if (ov == m_proReelOverlays.end())
            ov = m_proReelOverlays.emplace(&reel, ProReelDisplayOverlay(reel, designW, designH)).first;
         const ProImage& overlay = ov->second;
         if (!overlay.Empty())
         {
            const int locX = reel.m_locX - 1;
            const int xi = static_cast<int>(std::nearbyint(locX + digit * (designW + reel.m_spacing / 2.0)));
            const int sourceX = std::clamp(xi - locX, 0, overlay.w - 1);
            const int sourceWidth = std::min(designW, overlay.w - sourceX);
            if (sourceWidth > 0)
               ProDrawScaled(flat, overlay, sourceX, 0.0, sourceWidth, overlay.h, 0.0, 0.0, targetW, targetH, false);
         }
      }
      if (reel.m_hasTransform)
      {
         ProImage out;
         int bx, by;
         ProWarp(flat, quad, bx, by, out);
         UpdateTexture(&cache.tex, out.w, out.h, VPXTEXFMT_sRGBA8, out.px.data());
         cache.x = static_cast<float>(bx);
         cache.y = static_cast<float>(by);
         cache.w = static_cast<float>(out.w);
         cache.h = static_cast<float>(out.h);
      }
      else
      {
         UpdateTexture(&cache.tex, flat.w, flat.h, VPXTEXFMT_sRGBA8, flat.px.data());
         cache.w = static_cast<float>(flat.w);
         cache.h = static_cast<float>(flat.h);
      }
   }
   if (cache.tex == nullptr)
      return;
   if (reel.m_hasTransform)
      ctx->DrawImage(ctx, cache.tex, 1.f, 1.f, 1.f, 1.f, 0.f, 0.f, cache.w, cache.h, 0.f, 0.f, 0.f, ContentX(cache.x), ContentY(cache.y), cache.w, cache.h);
   else
      ctx->DrawImage(ctx, cache.tex, 1.f, 1.f, 1.f, 1.f, 0.f, 0.f, cache.w, cache.h, 0.f, 0.f, 0.f, ContentX(x), ContentY(static_cast<float>(reel.m_locY)), width, static_cast<float>(reel.m_height));
}

// B2S Pro: formBackglass ReelGapOverlay. The strip between digits i and i + 1 draws the columns of the
// display wide 3D window overlay that no digit covers (the same overlay and column rounding as
// DrawProReelDigit), on the gap box or, for a rotated/perspective score, warped onto the gap's quad.
void B2SRenderer::DrawProReelGaps(VPXRenderContext2D* ctx, const B2SScore& reel, float width)
{
   if (reel.m_width <= 2)
      return;
   const int digits = std::max(1, reel.m_digits);
   const int designW = std::max(1, static_cast<int>(std::nearbyint((reel.m_width - (digits - 1) * reel.m_spacing / 2.0) / digits)));
   const int designH = std::max(1, reel.m_height);
   auto ov = m_proReelOverlays.find(&reel);
   if (ov == m_proReelOverlays.end())
      ov = m_proReelOverlays.emplace(&reel, ProReelDisplayOverlay(reel, designW, designH)).first;
   const ProImage& overlay = ov->second;
   if (overlay.Empty())
      return;
   const int locX = reel.m_locX - 1;
   for (int i = 0; i < digits - 1; i++)
   {
      const int xi = static_cast<int>(std::nearbyint(locX + i * (designW + reel.m_spacing / 2.0)));
      const int xn = static_cast<int>(std::nearbyint(locX + (i + 1) * (designW + reel.m_spacing / 2.0)));
      const int sourceX = std::clamp(xi - locX + designW, 0, overlay.w);
      const int sourceEnd = std::clamp(xn - locX, sourceX, overlay.w);
      if (sourceEnd <= sourceX)
         continue;
      ProDigitCache& cache = m_proReelGaps[{ &reel, i }];
      if (!cache.valid)
      {
         cache.valid = true;
         ProQuad quad {};
         int targetW = sourceEnd - sourceX, targetH = designH;
         bool drawable = true;
         if (reel.m_hasTransform)
         {
            const ProQuad displayQuad = ProTransformOf(reel).DisplayQuad(static_cast<float>(reel.m_locX), static_cast<float>(reel.m_locY), static_cast<float>(reel.m_width), static_cast<float>(reel.m_height));
            const float digitU0 = (1.f + static_cast<float>(i) * (static_cast<float>(designW) + static_cast<float>(reel.m_spacing) / 2.f)) / static_cast<float>(reel.m_width);
            const float gapU0 = std::min(1.f, digitU0 + static_cast<float>(designW) / static_cast<float>(reel.m_width));
            const float gapU1 = std::min(1.f, (1.f + static_cast<float>(i + 1) * (static_cast<float>(designW) + static_cast<float>(reel.m_spacing) / 2.f)) / static_cast<float>(reel.m_width));
            drawable = gapU1 > gapU0;
            if (drawable)
            {
               quad = ProSubQuad(displayQuad, gapU0, gapU1, 1.f / static_cast<float>(reel.m_height), 1.f);
               ProContentSize(quad, targetW, targetH);
            }
         }
         if (drawable)
         {
            ProImage flat;
            flat.Allocate(targetW, targetH);
            ProDrawScaled(flat, overlay, sourceX, 0.0, sourceEnd - sourceX, overlay.h, 0.0, 0.0, targetW, targetH, false);
            if (reel.m_hasTransform)
            {
               ProImage out;
               int bx, by;
               ProWarp(flat, quad, bx, by, out);
               UpdateTexture(&cache.tex, out.w, out.h, VPXTEXFMT_sRGBA8, out.px.data());
               cache.x = static_cast<float>(bx);
               cache.y = static_cast<float>(by);
               cache.w = static_cast<float>(out.w);
               cache.h = static_cast<float>(out.h);
            }
            else
            {
               UpdateTexture(&cache.tex, flat.w, flat.h, VPXTEXFMT_sRGBA8, flat.px.data());
               cache.w = static_cast<float>(flat.w);
               cache.h = static_cast<float>(flat.h);
            }
         }
      }
      if (cache.tex == nullptr)
         continue;
      if (reel.m_hasTransform)
         ctx->DrawImage(ctx, cache.tex, 1.f, 1.f, 1.f, 1.f, 0.f, 0.f, cache.w, cache.h, 0.f, 0.f, 0.f, cache.x, cache.y, cache.w, cache.h);
      else
      {
         // Same placement as the digits: digit i ends at x + width, digit i + 1 starts at the next x
         const float gapX0 = static_cast<float>(reel.m_locX) + static_cast<float>(i) * (width + 0.5f * static_cast<float>(reel.m_spacing)) + width;
         const float gapX1 = static_cast<float>(reel.m_locX) + static_cast<float>(i + 1) * (width + 0.5f * static_cast<float>(reel.m_spacing));
         if (gapX1 > gapX0)
            ctx->DrawImage(ctx, cache.tex, 1.f, 1.f, 1.f, 1.f, 0.f, 0.f, cache.w, cache.h, 0.f, 0.f, 0.f, gapX0, static_cast<float>(reel.m_locY), gapX1 - gapX0, static_cast<float>(reel.m_height));
      }
   }
}


bool B2SRenderer::RenderBackglass(VPXRenderContext2D* ctx, B2SServer* server)
{
   // Update to latest settings state
   m_grillCut = showGrillProp_Get() ? 0.f : static_cast<float>(m_b2s->m_grillHeight);

   const VPXTextureInfo* bgTexInfo = nullptr;
   if (m_b2s->m_backglassOffImage.m_image)
      bgTexInfo = GetTextureInfo(m_b2s->m_backglassOffImage.m_image);
   else if (m_b2s->m_backglassImage.m_image)
      bgTexInfo = GetTextureInfo(m_b2s->m_backglassImage.m_image);
   m_b2sWidth = bgTexInfo ? static_cast<float>(bgTexInfo->width) : 1024.f;
   m_b2sHeight = (bgTexInfo ? static_cast<float>(bgTexInfo->height) : 768.f) - m_grillCut;

   // KeepAspectRatio: the source surface is padded to the window shape, the content keeps its shape on it
   FitSurfaceToWindow(ctx, m_b2sWidth, m_b2sHeight);

   // Update animations
   auto now = std::chrono::steady_clock::now();
   float elapsed = std::min(1.f, static_cast<float>(static_cast<double>((now - m_lastBackglassRenderTick).count()) / 1000000000.0));
   m_lastBackglassRenderTick = now;
   const B2SDualMode dualMode = ActiveDualMode();
   DispatchScriptTriggers(server);
   UpdateAnimations(m_b2s->m_backglassAnimations, elapsed, server, dualMode);

   // Draw background
   B2SImage& onImage = m_b2s->m_backglassOnImage;
   {
      const float prev = onImage.m_romProbe;
      onImage.m_romUpdater();
      if (server != nullptr && prev >= 0.f && onImage.m_romProbe != prev)
         onImage.m_romStamp = server->NextStamp();
   }
   // The lit background is also driven by script writes on its lamp channel (reference MyB2SSetData):
   // the most recent write between script and ROM wins, RomInverted is never applied to this channel
   uint64_t onLampStamp = 0;
   float onLampState = 0.f;
   if (server != nullptr && onImage.m_romIdType == B2SRomIDType::Lamp && onImage.m_romId > 0 && server->GetScriptedLampState(onImage.m_romId, onLampState, &onLampStamp)
      && onLampStamp > onImage.m_romStamp)
      onImage.m_brightness = (onLampState != 0.f) ? 1.f : 0.f;
   else if (onImage.m_romProbe >= 0.f)
      onImage.m_brightness = onImage.m_romProbe;

   // The canvas: unlit and lit backglass images (B2S Pro draws it at its layer position)
   const auto drawCanvas = [&]()
   {
      if (onImage.m_image == nullptr || onImage.m_brightness < 1.f)
      {
         // The unlit variant is the OffImage when the file provides one (the reference's DarkImage)
         const B2SImage& darkImage = m_b2s->m_backglassOffImage.m_image ? m_b2s->m_backglassOffImage : m_b2s->m_backglassImage;
         if (darkImage.m_image)
         {
            const VPXTextureInfo* texInfo = GetTextureInfo(darkImage.m_image);
            ctx->DrawImage(ctx, darkImage.m_image, 1.f, 1.f, 1.f, 1.f, 0.f, m_grillCut, static_cast<float>(texInfo->width), static_cast<float>(texInfo->height) - m_grillCut, 0.f, 0.f,
               0.f, // No rotation
               ContentX(0.f), ContentY(0.f), static_cast<float>(texInfo->width), static_cast<float>(texInfo->height) - m_grillCut);
         }
      }
      if (onImage.m_image)
      {
         const VPXTextureInfo* texInfo = GetTextureInfo(onImage.m_image);
         ctx->DrawImage(ctx, onImage.m_image, 1.f, 1.f, 1.f, onImage.m_brightness, 0.f, m_grillCut, static_cast<float>(texInfo->width), static_cast<float>(texInfo->height) - m_grillCut, 0.f,
            0.f, 0.f, // No rotation
            ContentX(0.f), ContentY(0.f), static_cast<float>(texInfo->width), static_cast<float>(texInfo->height) - m_grillCut);
      }
   };

   m_drawingBackglass = true;
   m_backglassDark = !(onImage.m_image != nullptr && onImage.m_brightness >= 0.5f);
   if (m_b2s->m_unifiedLayerDeck)
   {
      // B2S Pro: unified layer deck
      if (m_b2s->m_hasCanvasBacking)
         FillBacking(ctx, m_b2s->m_canvasBacking);
      RenderProDeck(ctx, server, m_b2s->m_backglassIlluminations, m_b2s->m_backglassScores, elapsed, dualMode, m_b2s->m_canvasLayerPosition, true, drawCanvas);
   }
   else
   {
      drawCanvas();
      // Draw illuminations, scores and DMD overlays
      RenderBulbs(ctx, server, m_b2s->m_backglassIlluminations, elapsed, dualMode);
      RenderScores(ctx, server, m_b2s->m_backglassScores, elapsed);
   }
   const VPXTexture backImage = m_b2s->m_backglassImage.m_image ? m_b2s->m_backglassImage.m_image
      : m_b2s->m_backglassOffImage.m_image                      ? m_b2s->m_backglassOffImage.m_image
                                                                : m_b2s->m_backglassOnImage.m_image;
   RenderDisplays(ctx, server, 0, backImage);

   RenderAspectPadding(ctx);

   return true;
}

bool B2SRenderer::RenderScoreView(VPXRenderContext2D* ctx, B2SServer* server)
{
   if (m_b2s->m_dmdImage.m_image == nullptr && m_b2s->m_dmdIlluminations.empty())
      return false;

   // Update to latest settings state
   // KeepAspectRatio: the source surface is padded to the window shape, the content keeps its shape on it
   FitSurfaceToWindow(ctx, m_dmdWidth, m_dmdHeight);

   // Update animations
   auto now = std::chrono::steady_clock::now();
   float elapsed = std::min(1.f, static_cast<float>(static_cast<double>((now - m_lastDmdRenderTick).count()) / 1000000000.0));
   m_lastDmdRenderTick = now;
   DispatchScriptTriggers(server);
   UpdateAnimations(m_b2s->m_dmdAnimations, elapsed, server, ActiveDualMode());
   // Draw background
   const auto drawCanvas = [&]()
   {
      if (m_b2s->m_dmdImage.m_image)
         ctx->DrawImage(ctx, m_b2s->m_dmdImage.m_image, 1.f, 1.f, 1.f, 1.f,
            0.f, 0.f, m_dmdWidth, m_dmdHeight,
            0.f, 0.f, 0.f, // No rotation
            ContentX(0.f), ContentY(0.f), m_dmdWidth, m_dmdHeight);
   };

   m_drawingBackglass = false;
   if (m_b2s->m_unifiedLayerDeck)
   {
      // B2S Pro: the DMD image sits at DMDCanvasLayerPosition only when that position is set and the
      // DMD has ranked items (formDMD OnPaintBackground), otherwise it is at the back
      const bool dmdRanked = std::ranges::any_of(m_b2s->m_dmdIlluminations, [](const std::unique_ptr<B2SBulb>& bulb) { return bulb->m_zOrder > 0; });
      const int canvasRank = (m_b2s->m_dmdCanvasLayerPosition > 0 && dmdRanked) ? m_b2s->m_dmdCanvasLayerPosition : INT_MIN;
      RenderProDeck(ctx, server, m_b2s->m_dmdIlluminations, m_b2s->m_dmdScores, elapsed, B2SDualMode::Both, canvasRank, false, drawCanvas);
   }
   else
   {
      drawCanvas();
      // Draw illuminations, scores and DMD overlays
      RenderBulbs(ctx, server, m_b2s->m_dmdIlluminations, elapsed, B2SDualMode::Both); // Reference does not dual-filter score view bulbs
      RenderScores(ctx, server, m_b2s->m_dmdScores, elapsed);
   }
   RenderDisplays(ctx, server, 1, m_b2s->m_dmdImage.m_image);

   RenderAspectPadding(ctx);

   return true;
}

// B2S Pro: unified layer deck (formBackglass OnPaint / DrawUnifiedLayersAboveScore and the formDMD
// equivalents). Every item has a positive unique rank; the canvas image sits at canvasRank and each
// score display at its ZOrder. The reference paints the form (items behind the canvas, canvas,
// items above it) and then each score display redraws, over itself, what ranks above it. That gives
// the same picture as drawing everything once in rank order, with, at an equal rank, items first,
// then the score display, then the canvas (an item or a display ranked at the canvas position is
// behind it). Drawing once also keeps translucent items from being blended twice.
// Backglass items without a rank keep their place at the bottom (rank 0); on the DMD the reference
// draws them after the ranked items, so they go on top of the ranked items there.
void B2SRenderer::RenderProDeck(VPXRenderContext2D* ctx, B2SServer* server, const vector<std::unique_ptr<B2SBulb>>& bulbs, const B2SScores& scores, float elapsed,
   B2SDualMode dualMode, int canvasRank, bool backglass, const std::function<void()>& drawCanvas)
{
   struct Key
   {
      int64_t rank;
      int kind; // 0 = item, 1 = score display, 2 = canvas
      bool operator<(const Key& o) const { return rank != o.rank ? rank < o.rank : kind < o.kind; }
   };
   if (m_proRuntime)
      m_proRuntime->Update(server, bulbs, elapsed);
   vector<B2SBulb*> drawn;
   drawn.reserve(bulbs.size());
   for (const auto& bulb : bulbs)
   {
      if (!IsBulbRendered(*bulb, dualMode))
         continue;
      UpdateBulb(server, *bulb, elapsed);
      drawn.push_back(bulb.get());
   }
   const auto bulbKey = [backglass](const B2SBulb* bulb) -> Key
   {
      if (bulb->m_zOrder > 0)
         return { bulb->m_zOrder, 0 };
      return { backglass ? 0 : static_cast<int64_t>(INT_MAX) + 1, 0 };
   };
   std::stable_sort(drawn.begin(), drawn.end(), [&bulbKey](const B2SBulb* a, const B2SBulb* b) { return bulbKey(a) < bulbKey(b); });
   const Key canvasKey { canvasRank, 2 };
   // B2S Pro dynamic front lights that show their light (not on a separate layer) first draw their own
   // image at the start of their deck segment (DrawIlluminationDeckSegment first pass): behind the
   // canvas, or just above it. Backglass only (the DMD draws dynamic lights as plain images).
   if (backglass)
   {
      vector<B2SBulb*> behindBases, frontBases;
      for (B2SBulb* bulb : drawn)
         if (bulb->m_pro && bulb->m_pro->dynamicFrontLighting && bulb->m_pro->dynamicFrontShowLight && !bulb->m_pro->separateLightLayer && !bulb->m_isImageSnippit)
            (bulbKey(bulb) < canvasKey ? behindBases : frontBases).push_back(bulb);
      for (B2SBulb* bulb : behindBases)
         if (bulb->m_brightness > 0.f)
            DrawProImage(ctx, *bulb, bulb->m_image);
      m_frontDynamicBases = std::move(frontBases);
   }
   else
      m_frontDynamicBases.clear();
   bool frontBasesDrawn = false;
   const auto drawFrontBases = [&]()
   {
      if (frontBasesDrawn)
         return;
      frontBasesDrawn = true;
      for (B2SBulb* bulb : m_frontDynamicBases)
         if (bulb->m_brightness > 0.f)
            DrawProImage(ctx, *bulb, bulb->m_image);
   };
   size_t next = 0;
   bool canvasDrawn = false;
   const auto drawBelow = [&](const Key& limit)
   {
      for (;;)
      {
         const bool bulbReady = next < drawn.size() && bulbKey(drawn[next]) < limit;
         const bool canvasReady = !canvasDrawn && canvasKey < limit;
         if (!bulbReady && !canvasReady)
            return;
         if (bulbReady && (!canvasReady || bulbKey(drawn[next]) < canvasKey))
         {
            if (canvasDrawn)
               drawFrontBases();
            DrawBulb(ctx, *drawn[next++]);
         }
         else
         {
            drawCanvas();
            canvasDrawn = true;
            drawFrontBases();
         }
      }
   };
   const std::function<void(const B2SScore&)> beforeScore = [&drawBelow](const B2SScore& score) { drawBelow({ score.m_zOrder, 1 }); };
   RenderScores(ctx, server, scores, elapsed, &beforeScore);
   drawBelow({ INT64_MAX, 3 });
}

// B2S Pro: solid backing under transparent canvas pixels, drawn from a 1x1 texture of the exact color
void B2SRenderer::FillBacking(VPXRenderContext2D* ctx, const vec4& color)
{
   const uint8_t pixel[4] = { static_cast<uint8_t>(std::lround(color.x * 255.f)), static_cast<uint8_t>(std::lround(color.y * 255.f)),
      static_cast<uint8_t>(std::lround(color.z * 255.f)), 255 };
   if (m_backingTex == nullptr || memcmp(pixel, m_backingPixel, 4) != 0)
   {
      memcpy(m_backingPixel, pixel, 4);
      UpdateTexture(&m_backingTex, 1, 1, VPXTEXFMT_sRGBA8, m_backingPixel);
   }
   ctx->DrawImage(ctx, m_backingTex, 1.f, 1.f, 1.f, 1.f, 0.f, 0.f, 1.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, ctx->srcWidth, ctx->srcHeight);
}

// Renders the <DMD> displays of the document targeted at this view, growing the overlay pool on
// demand (a display presence in the document means enabled). Each display keeps the overlay of its
// document index whatever its target, so that the overlay state (glass texture, uploaded frame,
// bounds search) is not shared between the backglass and score view displays
void B2SRenderer::RenderDisplays(VPXRenderContext2D* ctx, B2SServer* server, int target, VPXTexture backImage)
{
   vector<B2SDMDDisplay>& displays = server->GetOverlayDoc().displays;
   for (size_t i = 0; i < displays.size(); i++)
   {
      if (displays[i].target != target)
         continue;
      while (i >= m_dmdOverlays.size())
         m_dmdOverlays.emplace_back(std::make_unique<DMDOverlay::Renderer>(m_vpxApi, m_resURIResolver));
      m_dmdOverlays[i]->SetViewOffset(m_layoutOX, m_layoutOY); // Placed with the KeepAspectRatio placement
      m_dmdOverlays[i]->Render(ctx, displays[i], backImage);
   }
}
}
