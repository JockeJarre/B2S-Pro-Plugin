// license:GPLv3+

// B2S Pro: attributes of an Illumination/Bulb element of a B2S Pro file (Illumination/@UnifiedLayerDeck = 1),
// read the same way as the B2S Pro Server (formBackglass.LoadB2SData), with its defaults and clamps.
// Only created for bulbs of B2S Pro files; legacy bulbs have no B2SProBulbData.

#pragma once

#include "common.h"
#include "tinyxml2/tinyxml2.h"

#include <functional>
#include <string>
#include <vector>

namespace B2S {

struct ProPoint
{
   float x = 0.f;
   float y = 0.f;
};

struct ProRect
{
   float x = 0.f;
   float y = 0.f;
   float w = 0.f;
   float h = 0.f;
   bool IsEmpty() const { return w <= 0.f || h <= 0.f; }
};

struct ProSwitchZone
{
   ProRect bounds;
   int switchId = 0;
   float angle = 0.f;
   string soundName;
   float soundVolume = 1.f;
};

struct ProLauncher
{
   int triggerType = 1;
   int triggerId = 0;
   ProPoint origin;
   float angle = -90.f;
   float strength = 900.f;
   float randomAngle = 0.f;
   float randomStrength = 0.f;
   float captureRadius = 45.f;
   bool followPivot = false;
};

// Rotation stop behaviours (B2SPictureBox.eSnippitRotationStopBehaviour)
enum class ProStopBehaviour
{
   SpinOff = 0,
   StopImmediatelly = 1,
   RunAnimationTillEnd = 2,
   RunAnimationToFirstStep = 3,
};

// Motion path state of a B2S Pro picture (B2SPictureBox motion members), authored coordinates
struct ProMotion
{
   bool active = false; // The picture takes part in motion (path, exit, grid, sequence or physics)
   float x = 0.f, y = 0.f, w = 0.f, h = 0.f; // Current rectangle (RectangleF)
   float startW = 0.f, startH = 0.f; // motionPathStartRectangle size
   VPXTexture image = nullptr; // BackgroundImage (sequences reassign images between members)
   vector<ProPoint> authoredPoints; // _motionPathPoints (swapped between sequence members)

   bool timerOn = false;
   bool paused = false;
   double clockMs = 0.0;
   int activeDuration = 3000;
   int startDelay = 0;
   bool activeLoop = false;
   bool externalShift = false;
   vector<ProPoint> runtimePoints;
   vector<ProPoint> entryPoints;
   vector<ProPoint> exitPoints;
   bool runtimePrepared = false;
   ProPoint respawnPoint;
   ProPoint extStartVel, extEndVel, extLastVel;
   float arrivalSpeed = 0.f;
   bool launchReported = true;
   float launchProgress = 1.f;
   int queuedStarts = 0;
   float rollAngle = 0.f; // MotionPathRollAngle

   // Sequence feeder (persistent source shown at the path start)
   bool sourceAnchor = false;
   bool sourceVisible = false;
   ProRect sourceBounds; // Empty until the anchor first moves
   VPXTexture sourceImage = nullptr;
   float sourceRollAngle = 0.f;
   bool respawnActive = false;
   ProRect respawnBounds, respawnStart, respawnEnd;
   double respawnClockMs = 0.0;

   // Trough collision knock
   bool collisionOn = false;
   double collisionClockMs = 0.0;
   ProRect collisionBase;
   ProPoint collisionDir;
   float collisionAmp = 0.f;
   int collisionDelay = 0;

   // Sequence membership and its event subscriptions (AddHandler/RemoveHandler)
   int sequence = -1;
   int sequenceOrder = 0;
   bool subEntry = false, subLaunch = false, subRemoval = false, subDrain = false, subRespawn = false;
};

// Runtime state of a B2S Pro bulb (render thread only), see pro/B2SProRuntime.cpp
struct ProBulbRuntime
{
   ProMotion motion;
   // Native rotation and pivot pictures: their visibility and angle are owned by the B2S Pro runtime
   // (the reference routes their trigger to the rotation, not to the visibility)
   bool driven = false;
   bool visible = false;
   float angle = 0.f; // RotationAngle (degrees, clockwise positive)

   // Native one-image rotation (B2SPictureBox Start/Stop/SetNativeRotationState, NativeRotationTimer_Tick)
   bool nativeRotation = false;
   bool autoStartRotation = false;
   bool commandedOn = false;
   bool running = false;
   bool runTillEnd = false;
   bool runToFirstStep = false;
   double slowDownSteps = 0.0;
   float progress = 0.f;
   int steps = 24;
   int interval = 50;

   // Pivot rotation (SetPivotTriggerState / SetPivotRotationTarget / PivotRotationTimer_Tick)
   bool pivotRotation = false;
   bool automaticOscillation = false;
   bool automaticActive = false;
   bool moving = false;
   bool startupPivot = false;
   float downAngle = 0.f;
   float upAngle = -30.f;
   int moveDuration = 80;
   float startAngle = 0.f;
   float targetAngle = 0.f;
   int tweenDuration = 80;
   double tweenElapsedMs = 0.0;

   // Trigger source (native rotation trigger or pivot trigger)
   int triggerRomIdType = 0; // B2SRomIDType value: 1 lamp, 2 solenoid, 3 GI, 5 switch
   int triggerId = 0;
   bool triggerFromScript = false; // B2SSetData lamp-id writes also drive it
   bool triggerIsPivot = false;
   float romValue = -1.f;
   float lastRomValue = -1.f;
   uint64_t lastScriptStamp = 0;
   uint64_t lastGroupStamp = 0;
   std::function<void()> romUpdater = []() { };
   bool started = false;

   // Physics pivot (B2SPictureBox physics members)
   bool physicsPivot = false; // Advanced in the physics steps (B2SData.IsPhysicsPivot)
   bool physicsMoving = false; // physicsPivotMoving
   double physicsElapsedMs = 0.0;
   float physicsAngularVelocity = 0.f; // Degrees per second
   float physicsPreviousAngle = 0.f;
   int responseState = 0;
   double responseElapsedMs = 0.0;
   float responseStartAngle = 0.f;
   float responseRestAngle = 0.f;
   vector<ProPoint> collisionContour; // PhysicsCollisionContour (mask pixel coordinates)
   int collisionMaskW = 0;
   int collisionMaskH = 0;
};

struct B2SProBulbData
{
   explicit B2SProBulbData(const tinyxml2::XMLElement& node);

   ProBulbRuntime rt;
   string name; // Name attribute (illumination group)

   // Identity
   int id = 0;              // ID attribute (the reference names the control "PictureBox" & ID)
   string parent;           // "Backglass" or "DMD"
   bool isImageSnippit = false;
   int snippitType = 0;     // 0 standard, 1 self rotating, 2 mech rotating

   // ROM binding as resolved by the reference (B2SID wins; a native rotation trigger replaces it)
   int romId = 0;
   int romIdType = 0;       // 0 none, 1 lamp, 2 solenoid, 3 GI, 4 mech, 5 switch
   int romIdValue = 0;
   bool romInverted = false;
   int initialState = 0;

   int flasherPulseDuration = 0; // 0 = off, else 50..5000 ms

   // Full resolution snippets
   bool infinityAnimationFrame = false; // Name starts with PA_InfinityLights
   bool fullResolutionSnippet = false;
   bool useRuntimeSnippetImage = false;

   // Native one-image rotation
   bool nativeRotation = false;
   bool automaticRotationContinuous = false;
   int rotateSteps = 0;      // NativeRotationSteps = max(2, rotateSteps)
   int rotateInterval = 0;   // NativeRotationInterval = max(10, rotateInterval)
   int rotationDirection = 0; // 0 clockwise, 1 anticlockwise
   ProStopBehaviour stopBehaviour = ProStopBehaviour::SpinOff;

   // Pivot / external pivot
   string externalGridName;
   bool externalGridRepresentative = false;
   int externalGridDuration = 45;
   string externalPivotGroup;
   bool externalPivotRepresentative = false;
   float externalPivotAngle = 0.f;
   int pivotDuration = 80;   // ExternalPivotDuration / PivotDuration
   float pivotX = 0.5f;
   float pivotY = 0.5f;
   float pivotTipX = 0.9f;
   float pivotTipY = 0.5f;
   bool pivotAnimation = false;
   bool pivotAutomaticOscillation = false;
   bool pivotPhysicsEnabled = false;
   float pivotPhysicsWeight = 1.f;
   float pivotPhysicsFriction = 1.25f;
   float pivotPhysicsGrip = 0.5f;
   int pivotPhysicsResponseMode = 0;
   float pivotPhysicsKnockdownAngle = 90.f;
   int pivotPhysicsKnockdownDuration = 180;
   int pivotPhysicsResetDelay = 1000;
   float pivotDownAngle = 0.f;
   float pivotUpAngle = -30.f;
   string pivotDownTrigger;
   string pivotUpTrigger;
   int pivotTriggerType = 1;
   int pivotTriggerId = 0;
   float pivotRandomStrength = 0.f;

   // Physics ball
   bool physicsBall = false;
   string physicsFlipperName;
   vector<string> physicsPivotTargetNames;
   ProRect physicsBounds;
   float physicsGravity = 650.f;
   float physicsFlipperStrength = 1.f;
   float physicsBoundaryBounce = 0.12f;
   vector<ProPoint> physicsFloorPoints;
   vector<vector<ProPoint>> physicsBoundaryPaths;
   vector<vector<float>> physicsBoundarySegmentBounces;
   vector<ProRect> physicsObstacles;
   vector<float> physicsObstacleBounces;
   vector<ProSwitchZone> physicsSwitchZones;
   bool hasPhysicsLauncher = false;
   ProLauncher physicsLauncher;
   vector<uint8_t> physicsSelectionMaskPng; // PhysicsSelectionMaskData (PNG, alpha is the mask)

   // Motion path
   int motionPathDuration = 3000;
   bool motionPathLoop = false;
   int motionPathSolenoidId = 0;
   int motionPathLampId = 0;
   int motionPathB2SId = 0;
   int motionPathStopB2SId = 0;
   int motionPathResumeB2SId = 0;
   bool motionPathQueueTriggers = false;
   bool motionPathRollEnabled = false;
   string motionPathSequenceGroup;
   int motionPathSequenceOrder = 0;
   bool motionPathDrainAll = false;
   bool motionPathGravityDrop = false;
   bool motionPathRespawnEnabled = false;
   ProPoint motionPathRespawnPoint;
   int motionPathRespawnDuration = 350;
   int motionPathExitDuration = 3000;
   int motionPathRemoveSolenoidId = 0;
   int motionPathRemoveLampId = 0;
   int motionPathRemoveB2SId = 0;
   int motionPathSwitchId = 0;
   int motionPathStopSwitchId = 0;
   int motionPathStopLampId = 0;
   int motionPathStopSolenoidId = 0;
   int motionPathResumeSwitchId = 0;
   int motionPathResumeLampId = 0;
   int motionPathResumeSolenoidId = 0;
   int motionPathRemoveSwitchId = 0;
   vector<ProPoint> motionPathPoints;
   vector<ProPoint> motionPathExitPoints;

   // Dynamic front lighting
   bool artworkPixelLighting = false;
   bool separateLightLayer = false;
   bool dynamicFrontLighting = false; // True only with a decoded light field
   bool dynamicFrontShowLight = false;
   int dynamicFrontTargetSnippetId = 0;
   bool dynamicFrontTargetSelectionDefined = false;
   vector<int> dynamicFrontTargetSnippetIds;
   vector<uint8_t> dynamicFrontLightFieldPng;
   int dynamicLightExposure = 100;
   int dynamicLightColorR = 255, dynamicLightColorG = 255, dynamicLightColorB = 255;
   int dynamicFlasherStyle = 1;
   int dynamicSaturation = 100;
   int dynamicHighlightProtection = 0;
   int dynamicDarkAreaLift = 0;
   int dynamicTransmissionContrast = 140;
   int dynamicLightTemperature = 4000;
};

// Parse helpers shared by the B2S Pro readers (invariant culture, tolerant like Single/Integer.TryParse)
bool ProParseFloat(std::string_view text, float& value);
bool ProParseInt(std::string_view text, int& value);
vector<ProPoint> ProParsePoints(std::string_view text); // "x,y;x,y;..."

}
