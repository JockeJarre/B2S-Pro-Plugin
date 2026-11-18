// license:GPLv3+

// B2S Pro: bulb attribute reader, a port of the Illumination/Bulb part of formBackglass.LoadB2SData
// (B2S Pro Server). Defaults and clamps follow the reference line by line.

#include "B2SProData.h"

#include <algorithm>
#include <charconv>
#include <cmath>

namespace B2S {

static std::string_view Trim(std::string_view s)
{
   while (!s.empty() && (s.front() == ' ' || s.front() == '\t' || s.front() == '\r' || s.front() == '\n'))
      s.remove_prefix(1);
   while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r' || s.back() == '\n'))
      s.remove_suffix(1);
   return s;
}

bool ProParseFloat(std::string_view text, float& value)
{
   text = Trim(text);
   if (!text.empty() && text.front() == '+')
      text.remove_prefix(1);
   if (text.empty())
      return false;
   float parsed = 0.f;
   const auto res = std::from_chars(text.data(), text.data() + text.size(), parsed, std::chars_format::general);
   if (res.ec != std::errc() || res.ptr != text.data() + text.size())
      return false;
   value = parsed;
   return true;
}

bool ProParseInt(std::string_view text, int& value)
{
   text = Trim(text);
   if (!text.empty() && text.front() == '+')
      text.remove_prefix(1);
   if (text.empty())
      return false;
   int parsed = 0;
   const auto res = std::from_chars(text.data(), text.data() + text.size(), parsed);
   if (res.ec != std::errc() || res.ptr != text.data() + text.size())
      return false;
   value = parsed;
   return true;
}

static vector<std::string_view> Split(std::string_view text, char delimiter)
{
   vector<std::string_view> parts;
   size_t start = 0;
   for (;;)
   {
      const size_t pos = text.find(delimiter, start);
      if (pos == std::string_view::npos)
      {
         parts.push_back(text.substr(start));
         return parts;
      }
      parts.push_back(text.substr(start, pos - start));
      start = pos + 1;
   }
}

static bool IsBlank(std::string_view text) { return Trim(text).empty(); }

vector<ProPoint> ProParsePoints(std::string_view text)
{
   // ParseMotionPathPoints: "x,y;x,y", pairs that do not parse are skipped
   vector<ProPoint> points;
   if (IsBlank(text))
      return points;
   for (const std::string_view pair : Split(text, ';'))
   {
      const vector<std::string_view> parts = Split(pair, ',');
      if (parts.size() != 2)
         continue;
      ProPoint p;
      if (ProParseFloat(parts[0], p.x) && ProParseFloat(parts[1], p.y))
         points.push_back(p);
   }
   return points;
}

static const char* Attr(const tinyxml2::XMLElement& node, const char* name) { return node.Attribute(name); }
static bool Has(const tinyxml2::XMLElement& node, const char* name) { return node.Attribute(name) != nullptr; }
static bool IsOne(const tinyxml2::XMLElement& node, const char* name)
{
   const char* v = node.Attribute(name);
   return v != nullptr && std::string_view(v) == "1";
}
static void ReadInt(const tinyxml2::XMLElement& node, const char* name, int& value)
{
   if (const char* v = node.Attribute(name))
      ProParseInt(v, value);
}
static void ReadFloat(const tinyxml2::XMLElement& node, const char* name, float& value)
{
   if (const char* v = node.Attribute(name))
      ProParseFloat(v, value);
}
static string ReadTrimmed(const tinyxml2::XMLElement& node, const char* name)
{
   const char* v = node.Attribute(name);
   return v ? string(Trim(v)) : string();
}
static bool StartsWithNoCase(std::string_view s, std::string_view prefix)
{
   if (s.size() < prefix.size())
      return false;
   for (size_t i = 0; i < prefix.size(); i++)
      if (std::tolower(static_cast<unsigned char>(s[i])) != std::tolower(static_cast<unsigned char>(prefix[i])))
         return false;
   return true;
}
static vector<uint8_t> ReadBase64(const tinyxml2::XMLElement& node, const char* name)
{
   const char* v = node.Attribute(name);
   if (v == nullptr)
      return {};
   std::string_view view { v };
   return base64_decode(view.data(), view.size());
}

B2SProBulbData::B2SProBulbData(const tinyxml2::XMLElement& node)
{
   ReadInt(node, "ID", id);
   if (const char* v = Attr(node, "Parent"))
      parent = v;

   int romid = 0;
   int romidtype = 0;
   int romidvalue = 0;
   bool rominverted = false;
   if (Has(node, "B2SID"))
   {
      ReadInt(node, "B2SID", romid);
      ReadInt(node, "B2SValue", romidvalue);
      romidtype = 1;
   }
   else
   {
      ReadInt(node, "RomID", romid);
      ReadInt(node, "RomIDType", romidtype);
      rominverted = IsOne(node, "RomInverted");
   }
   ReadInt(node, "InitialState", initialState);
   if (Has(node, "FlasherPulseDuration"))
   {
      ReadInt(node, "FlasherPulseDuration", flasherPulseDuration);
      if (flasherPulseDuration > 0)
         flasherPulseDuration = std::clamp(flasherPulseDuration, 50, 5000);
   }
   name = Attr(node, "Name") ? Attr(node, "Name") : "";
   isImageSnippit = IsOne(node, "IsImageSnippit");
   artworkPixelLighting = IsOne(node, "ArtworkPixelLighting");
   separateLightLayer = IsOne(node, "SeparateLightLayer");
   const bool dynamicFrontLightingAttr = IsOne(node, "DynamicFrontLighting");
   const bool dynamicFrontShowLightAttr = IsOne(node, "DynamicFrontShowLight");
   if (Has(node, "DynamicFrontTargetSnippetID"))
   {
      ReadInt(node, "DynamicFrontTargetSnippetID", dynamicFrontTargetSnippetId);
      dynamicFrontTargetSnippetId = std::max(0, dynamicFrontTargetSnippetId);
   }
   dynamicFrontTargetSelectionDefined = Has(node, "DynamicFrontTargetSnippetIDs");
   if (dynamicFrontTargetSelectionDefined)
      for (const std::string_view token : Split(Attr(node, "DynamicFrontTargetSnippetIDs"), '|'))
      {
         int targetId = 0;
         if (ProParseInt(token, targetId) && targetId > 0 && std::find(dynamicFrontTargetSnippetIds.begin(), dynamicFrontTargetSnippetIds.end(), targetId) == dynamicFrontTargetSnippetIds.end())
            dynamicFrontTargetSnippetIds.push_back(targetId);
      }

   int picboxtype = 0;
   int picboxrotatesteps = 0;
   int picboxrotateinterval = 0;
   int picboxrotationdirection = 0;
   int picboxrotationstopbehaviour = 0;
   bool nativerotation = false;
   if (Has(node, "SnippitType"))
   {
      ReadInt(node, "SnippitType", picboxtype);
      if (Has(node, "SnippitRotatingSteps"))
         ReadInt(node, "SnippitRotatingSteps", picboxrotatesteps);
      else if (Has(node, "SnippitRotatingAngle"))
      {
         int angle = 0;
         if (ProParseInt(Attr(node, "SnippitRotatingAngle"), angle) && angle != 0)
            picboxrotatesteps = static_cast<int>(std::lround(360.0 / angle)); // CInt rounds to nearest even; exact for divisors of 360
      }
      ReadInt(node, "SnippitRotatingInterval", picboxrotateinterval);
      ReadInt(node, "SnippitRotatingDirection", picboxrotationdirection);
      ReadInt(node, "SnippitRotatingStopBehaviour", picboxrotationstopbehaviour);
   }
   if (picboxtype == 0 && Has(node, "AutomaticRotationEnabled"))
      nativerotation = IsOne(node, "AutomaticRotationEnabled");
   if (nativerotation && Has(node, "AutomaticRotationContinuous"))
      automaticRotationContinuous = IsOne(node, "AutomaticRotationContinuous");
   if (nativerotation && picboxtype != 2)
   {
      if (Has(node, "AutomaticRotationTriggerID"))
      {
         int v = 0;
         ReadInt(node, "AutomaticRotationTriggerID", v);
         romid = std::max(0, v);
      }
      if (Has(node, "AutomaticRotationTriggerType"))
      {
         int triggerType = 0;
         ReadInt(node, "AutomaticRotationTriggerType", triggerType);
         romidtype = triggerType == 2 ? 2 : (triggerType == 5 ? 5 : 1);
      }
      if (automaticRotationContinuous)
      {
         romid = 0;
         romidtype = 0;
      }
      if (Has(node, "AutomaticRotationSteps"))
      {
         int v = 0;
         ReadInt(node, "AutomaticRotationSteps", v);
         picboxrotatesteps = std::clamp(v, 4, 360);
      }
      if (Has(node, "AutomaticRotationInterval"))
      {
         int v = 0;
         ReadInt(node, "AutomaticRotationInterval", v);
         picboxrotateinterval = std::clamp(v, 10, 500);
      }
      ReadInt(node, "AutomaticRotationDirection", picboxrotationdirection);
      ReadInt(node, "AutomaticRotationStopBehaviour", picboxrotationstopbehaviour);
   }

   externalGridName = ReadTrimmed(node, "ExternalGridName");
   externalGridRepresentative = IsOne(node, "ExternalGridRepresentative");
   if (Has(node, "ExternalGridDuration"))
   {
      ReadInt(node, "ExternalGridDuration", externalGridDuration);
      externalGridDuration = std::clamp(externalGridDuration, 10, 1000);
   }
   externalPivotGroup = ReadTrimmed(node, "ExternalPivotGroup");
   externalPivotRepresentative = IsOne(node, "ExternalPivotRepresentative");
   ReadFloat(node, "ExternalPivotAngle", externalPivotAngle);
   ReadInt(node, "ExternalPivotDuration", pivotDuration);
   float externalPivotX = 0.5f, externalPivotY = 0.5f, externalPivotTipX = 0.9f, externalPivotTipY = 0.5f;
   ReadFloat(node, "ExternalPivotX", externalPivotX);
   ReadFloat(node, "ExternalPivotY", externalPivotY);
   pivotAnimation = IsOne(node, "PivotAnimation");
   pivotAutomaticOscillation = IsOne(node, "PivotAutomaticOscillation");
   const bool pivotPhysicsAttr = IsOne(node, "PivotPhysicsEnabled");
   if (pivotAnimation && pivotPhysicsAttr)
      nativerotation = true;
   ReadFloat(node, "PivotPhysicsWeight", pivotPhysicsWeight);
   ReadFloat(node, "PivotPhysicsFriction", pivotPhysicsFriction);
   ReadFloat(node, "PivotPhysicsGrip", pivotPhysicsGrip);
   ReadInt(node, "PivotPhysicsResponseMode", pivotPhysicsResponseMode);
   ReadFloat(node, "PivotPhysicsKnockdownAngle", pivotPhysicsKnockdownAngle);
   ReadInt(node, "PivotPhysicsKnockdownDuration", pivotPhysicsKnockdownDuration);
   ReadInt(node, "PivotPhysicsResetDelay", pivotPhysicsResetDelay);
   ReadFloat(node, "PivotX", externalPivotX);
   ReadFloat(node, "PivotY", externalPivotY);
   ReadFloat(node, "PivotTipX", externalPivotTipX);
   ReadFloat(node, "PivotTipY", externalPivotTipY);
   ReadFloat(node, "PivotDownAngle", pivotDownAngle);
   ReadFloat(node, "PivotUpAngle", pivotUpAngle);
   ReadInt(node, "PivotDuration", pivotDuration);
   pivotDownTrigger = ReadTrimmed(node, "PivotDownTrigger");
   pivotUpTrigger = ReadTrimmed(node, "PivotUpTrigger");
   pivotTriggerReverse = IsOne(node, "PivotTriggerReverse");
   pivotWeight = IsOne(node, "PivotWeight");
   ReadFloat(node, "PivotWeightX", pivotWeightX);
   ReadFloat(node, "PivotWeightY", pivotWeightY);
   ReadFloat(node, "PivotWeightStrength", pivotWeightStrength);
   pivotStrikeTarget = ReadTrimmed(node, "PivotStrikeTarget");
   ReadFloat(node, "PivotStrikeStrength", pivotStrikeStrength);
   pivotStrikeContact = ReadTrimmed(node, "PivotStrikeContact") != "0";
   pivotStrikeDirection = ReadTrimmed(node, "PivotStrikeDirection") == "-1" ? -1 : 1;
   ReadInt(node, "PivotTriggerType", pivotTriggerType);
   ReadInt(node, "PivotTriggerID", pivotTriggerId);

   physicsBall = IsOne(node, "PhysicsBall");
   physicsFlipperName = ReadTrimmed(node, "PhysicsFlipperName");
   if (Has(node, "PhysicsPivotTargetNames"))
   {
      for (const std::string_view token : Split(Attr(node, "PhysicsPivotTargetNames"), '|'))
      {
         const string target { Trim(token) };
         if (target.empty())
            continue;
         const bool dup = std::any_of(physicsPivotTargetNames.begin(), physicsPivotTargetNames.end(),
            [&target](const string& t) { return t.size() == target.size() && StartsWithNoCase(t, target); });
         if (!dup)
            physicsPivotTargetNames.push_back(target);
      }
   }
   else if (!physicsFlipperName.empty())
      physicsPivotTargetNames.push_back(physicsFlipperName);
   if (const char* v = Attr(node, "PhysicsBounds"))
   {
      // ParsePhysicsBounds: "left,top,right,bottom"
      const vector<std::string_view> parts = Split(v, ',');
      float l, t, r, b;
      if (!IsBlank(v) && parts.size() == 4 && ProParseFloat(parts[0], l) && ProParseFloat(parts[1], t) && ProParseFloat(parts[2], r) && ProParseFloat(parts[3], b) && r > l && b > t)
         physicsBounds = { l, t, r - l, b - t };
   }
   ReadFloat(node, "PhysicsGravity", physicsGravity);
   ReadFloat(node, "PhysicsFlipperStrength", physicsFlipperStrength);
   ReadFloat(node, "PhysicsBoundaryBounce", physicsBoundaryBounce);
   if (const char* v = Attr(node, "PhysicsFloorPoints"))
      physicsFloorPoints = ProParsePoints(v);
   if (const char* v = Attr(node, "PhysicsBoundaries"); v && !IsBlank(v))
      for (const std::string_view encoded : Split(v, '|'))
      {
         vector<ProPoint> path = ProParsePoints(encoded);
         if (path.size() >= 2)
            physicsBoundaryPaths.push_back(std::move(path));
      }
   if (const char* v = Attr(node, "PhysicsBoundarySegmentBounces"); v && !IsBlank(v))
      for (const std::string_view encodedPath : Split(v, '|'))
      {
         vector<float> segments;
         for (const std::string_view encodedSegment : Split(encodedPath, ','))
         {
            float parsed;
            segments.push_back(ProParseFloat(encodedSegment, parsed) ? std::clamp(parsed, -1.f, 3.f) : -1.f);
         }
         physicsBoundarySegmentBounces.push_back(std::move(segments));
      }
   if (const char* v = Attr(node, "PhysicsObstacles"); v && !IsBlank(v))
      for (const std::string_view encoded : Split(v, '|'))
      {
         const vector<std::string_view> parts = Split(encoded, ',');
         float x, y, w, h;
         if (parts.size() == 4 && ProParseFloat(parts[0], x) && ProParseFloat(parts[1], y) && ProParseFloat(parts[2], w) && ProParseFloat(parts[3], h) && w > 0.f && h > 0.f)
            physicsObstacles.push_back({ x, y, w, h });
      }
   if (const char* v = Attr(node, "PhysicsObstacleBounces"); v && !IsBlank(v))
      for (const std::string_view encoded : Split(v, ','))
      {
         float parsed;
         if (!ProParseFloat(encoded, parsed) || std::isnan(parsed) || std::isinf(parsed) || parsed < 0.f)
            physicsObstacleBounces.push_back(-1.f);
         else
            physicsObstacleBounces.push_back(std::min(3.f, parsed));
      }
   bool hasZones = false;
   if (const char* v = Attr(node, "PhysicsSwitchZones"))
   {
      hasZones = true;
      if (!IsBlank(v))
         for (const std::string_view encoded : Split(v, '|'))
         {
            const vector<std::string_view> parts = Split(encoded, ',');
            float x, y, w, h;
            int sw;
            if (parts.size() == 5 && ProParseFloat(parts[0], x) && ProParseFloat(parts[1], y) && ProParseFloat(parts[2], w) && ProParseFloat(parts[3], h) && ProParseInt(parts[4], sw) && w > 0.f
               && h > 0.f && sw >= 0)
            {
               ProSwitchZone zone;
               zone.bounds = { x, y, w, h };
               zone.switchId = std::min(255, sw);
               physicsSwitchZones.push_back(zone);
            }
         }
   }
   if (hasZones)
   {
      if (const char* v = Attr(node, "PhysicsSwitchAngles"); v && !IsBlank(v))
      {
         const vector<std::string_view> encoded = Split(v, '|');
         for (size_t i = 0; i < std::min(encoded.size(), physicsSwitchZones.size()); i++)
         {
            float angle;
            if (ProParseFloat(encoded[i], angle))
            {
               angle = std::fmod(angle, 360.f);
               if (angle > 180.f)
                  angle -= 360.f;
               if (angle <= -180.f)
                  angle += 360.f;
               physicsSwitchZones[i].angle = angle;
            }
         }
      }
      if (const char* v = Attr(node, "PhysicsSwitchSoundNames"))
      {
         const vector<std::string_view> encoded = Split(v, '|');
         for (size_t i = 0; i < std::min(encoded.size(), physicsSwitchZones.size()); i++)
            physicsSwitchZones[i].soundName = string(Trim(encoded[i]));
      }
      if (const char* v = Attr(node, "PhysicsSwitchSoundVolumes"))
      {
         const vector<std::string_view> encoded = Split(v, '|');
         for (size_t i = 0; i < std::min(encoded.size(), physicsSwitchZones.size()); i++)
         {
            int percent;
            if (ProParseInt(encoded[i], percent))
               physicsSwitchZones[i].soundVolume = std::clamp(static_cast<float>(percent) / 100.f, 0.f, 2.f);
         }
      }
   }
   if (IsOne(node, "PhysicsLauncherEnabled"))
   {
      hasPhysicsLauncher = true;
      physicsLauncher.followPivot = IsOne(node, "PhysicsLauncherFollowPivot");
      ReadInt(node, "PhysicsLauncherTriggerType", physicsLauncher.triggerType);
      ReadInt(node, "PhysicsLauncherTriggerID", physicsLauncher.triggerId);
      ReadFloat(node, "PhysicsLauncherX", physicsLauncher.origin.x);
      ReadFloat(node, "PhysicsLauncherY", physicsLauncher.origin.y);
      ReadFloat(node, "PhysicsLauncherAngle", physicsLauncher.angle);
      ReadFloat(node, "PhysicsLauncherStrength", physicsLauncher.strength);
      ReadFloat(node, "PhysicsLauncherRandomAngle", physicsLauncher.randomAngle);
      ReadFloat(node, "PhysicsLauncherRandomStrength", physicsLauncher.randomStrength);
      ReadFloat(node, "PhysicsLauncherCaptureRadius", physicsLauncher.captureRadius);
   }
   if (physicsBoundaryPaths.empty() && physicsFloorPoints.size() >= 2)
      physicsBoundaryPaths.push_back(physicsFloorPoints);

   if (isImageSnippit)
   {
      if (Has(node, "MotionPathDuration"))
      {
         ReadInt(node, "MotionPathDuration", motionPathDuration);
         motionPathDuration = std::clamp(motionPathDuration, 250, 30000);
      }
      if (const char* v = Attr(node, "MotionPathPoints"))
         motionPathPoints = ProParsePoints(v);
      motionPathLoop = IsOne(node, "MotionPathLoop");
      const auto readClamped = [&node](const char* attr, int& value, int hi)
      {
         if (Has(node, attr))
         {
            ReadInt(node, attr, value);
            value = std::clamp(value, 0, hi);
         }
      };
      readClamped("MotionPathSolenoidID", motionPathSolenoidId, 255);
      readClamped("MotionPathLampID", motionPathLampId, 255);
      readClamped("MotionPathB2SID", motionPathB2SId, 250);
      readClamped("MotionPathStopB2SID", motionPathStopB2SId, 250);
      readClamped("MotionPathResumeB2SID", motionPathResumeB2SId, 250);
      motionPathQueueTriggers = IsOne(node, "MotionPathQueueTriggers");
      motionPathRollEnabled = IsOne(node, "MotionPathRollEnabled");
      motionPathSequenceGroup = ReadTrimmed(node, "MotionPathSequenceGroup");
      readClamped("MotionPathSequenceOrder", motionPathSequenceOrder, 999);
      motionPathDrainAll = IsOne(node, "MotionPathDrainAll");
      motionPathGravityDrop = IsOne(node, "MotionPathGravityDrop");
      motionPathRespawnEnabled = IsOne(node, "MotionPathRespawnEnabled");
      if (motionPathRespawnEnabled)
      {
         ReadFloat(node, "MotionPathRespawnX", motionPathRespawnPoint.x);
         ReadFloat(node, "MotionPathRespawnY", motionPathRespawnPoint.y);
         ReadInt(node, "MotionPathRespawnDuration", motionPathRespawnDuration);
         motionPathRespawnDuration = std::clamp(motionPathRespawnDuration, 50, 5000);
      }
      if (Has(node, "MotionPathExitDuration"))
      {
         ReadInt(node, "MotionPathExitDuration", motionPathExitDuration);
         motionPathExitDuration = std::clamp(motionPathExitDuration, 250, 30000);
      }
      readClamped("MotionPathRemoveSolenoidID", motionPathRemoveSolenoidId, 255);
      readClamped("MotionPathRemoveLampID", motionPathRemoveLampId, 255);
      readClamped("MotionPathRemoveB2SID", motionPathRemoveB2SId, 250);
      if (const char* v = Attr(node, "MotionPathExitPoints"))
         motionPathExitPoints = ProParsePoints(v);
   }

   infinityAnimationFrame = isImageSnippit && StartsWithNoCase(name, "PA_InfinityLights");
   const bool runtimeAttr = Has(node, "RuntimeSnippitImage");
   fullResolutionSnippet = isImageSnippit && !infinityAnimationFrame && !nativerotation && picboxtype == 0 && runtimeAttr;
   useRuntimeSnippetImage = runtimeAttr && (fullResolutionSnippet || infinityAnimationFrame);

   if (dynamicFrontLightingAttr && Has(node, "DynamicFrontLightField"))
      dynamicFrontLightFieldPng = ReadBase64(node, "DynamicFrontLightField");
   dynamicFrontLighting = dynamicFrontLightingAttr && !dynamicFrontLightFieldPng.empty();
   dynamicFrontShowLight = dynamicFrontLighting && dynamicFrontShowLightAttr;
   if (dynamicFrontLighting)
   {
      const auto readClamp = [&node](const char* attr, int& value, int lo, int hi)
      {
         if (Has(node, attr))
         {
            ReadInt(node, attr, value);
            value = std::clamp(value, lo, hi);
         }
      };
      readClamp("DynamicLightExposure", dynamicLightExposure, 0, 1600);
      if (const char* v = Attr(node, "DynamicLightColor"))
      {
         const vector<std::string_view> parts = Split(v, '.');
         int r, g, b;
         if (parts.size() >= 3 && ProParseInt(parts[0], r) && ProParseInt(parts[1], g) && ProParseInt(parts[2], b))
         {
            dynamicLightColorR = std::clamp(r, 0, 255);
            dynamicLightColorG = std::clamp(g, 0, 255);
            dynamicLightColorB = std::clamp(b, 0, 255);
         }
      }
      ReadInt(node, "DynamicFlasherStyle", dynamicFlasherStyle);
      readClamp("DynamicSaturation", dynamicSaturation, 0, 200);
      readClamp("DynamicHighlightProtection", dynamicHighlightProtection, 0, 100);
      readClamp("DynamicDarkAreaLift", dynamicDarkAreaLift, 0, 100);
      readClamp("DynamicTransmissionContrast", dynamicTransmissionContrast, 0, 300);
      readClamp("DynamicLightTemperature", dynamicLightTemperature, 2000, 6500);
   }

   if (physicsBall && Has(node, "PhysicsSelectionMaskData"))
      physicsSelectionMaskPng = ReadBase64(node, "PhysicsSelectionMaskData");

   ReadFloat(node, "PivotRandomStrength", pivotRandomStrength);
   if (std::isnan(pivotRandomStrength) || std::isinf(pivotRandomStrength))
      pivotRandomStrength = 0.f;
   pivotRandomStrength = std::clamp(pivotRandomStrength, 0.f, 100.f);

   // Pivot triggers own both ON and OFF states: the image is not also routed through visibility switching
   romId = pivotAnimation ? 0 : romid;
   romIdType = romidtype;
   romIdValue = romidvalue;
   romInverted = rominverted;
   snippitType = picboxtype;
   nativeRotation = nativerotation;
   rotateSteps = picboxrotatesteps;
   rotateInterval = picboxrotateinterval;
   rotationDirection = picboxrotationdirection;
   stopBehaviour = static_cast<ProStopBehaviour>(std::clamp(picboxrotationstopbehaviour, 0, 3));
   pivotX = pivotAnimation ? externalPivotX : std::clamp(externalPivotX, 0.f, 1.f);
   pivotY = pivotAnimation ? externalPivotY : std::clamp(externalPivotY, 0.f, 1.f);
   pivotTipX = std::clamp(externalPivotTipX, 0.f, 1.f);
   pivotTipY = std::clamp(externalPivotTipY, 0.f, 1.f);
   pivotPhysicsEnabled = pivotPhysicsAttr && pivotAnimation;
   pivotPhysicsWeight = std::clamp(pivotPhysicsWeight, 0.1f, 100.f);
   pivotPhysicsFriction = std::clamp(pivotPhysicsFriction, 0.f, 20.f);
   pivotPhysicsGrip = std::clamp(pivotPhysicsGrip, 0.f, 1.f);
   pivotPhysicsResponseMode = std::clamp(pivotPhysicsResponseMode, 0, 2);
   pivotPhysicsKnockdownAngle = std::clamp(pivotPhysicsKnockdownAngle, -360.f, 360.f);
   pivotPhysicsKnockdownDuration = std::clamp(pivotPhysicsKnockdownDuration, 10, 5000);
   pivotPhysicsResetDelay = std::clamp(pivotPhysicsResetDelay, 0, 30000);
   const auto clampId = [&node](const char* attr, int& value)
   {
      if (Has(node, attr))
      {
         ReadInt(node, attr, value);
         value = std::clamp(value, 0, 255);
      }
   };
   clampId("MotionPathSwitchID", motionPathSwitchId);
   clampId("MotionPathStopSwitchID", motionPathStopSwitchId);
   clampId("MotionPathStopLampID", motionPathStopLampId);
   clampId("MotionPathStopSolenoidID", motionPathStopSolenoidId);
   clampId("MotionPathResumeSwitchID", motionPathResumeSwitchId);
   clampId("MotionPathResumeLampID", motionPathResumeLampId);
   clampId("MotionPathResumeSolenoidID", motionPathResumeSolenoidId);
   clampId("MotionPathRemoveSwitchID", motionPathRemoveSwitchId);
}

}
