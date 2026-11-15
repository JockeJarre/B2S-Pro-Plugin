// license:GPLv3+

// B2S Pro: physics balls, a port of PhysicsBallState (shared/PhysicsBallState.vb), the physics clock of
// B2SData and the physics members of B2SPictureBox (AdvancePhysicsPivot, ApplyPivotPhysicsAngularImpulse,
// ConfigurePhysicsCollisionOutline) of the B2S Pro Server. Simulation runs in authored backglass
// coordinates in fixed 1 ms steps.

#pragma once

#include "common.h"
#include "B2SDataModel.h"
#include "pro/B2SProData.h"
#include "pro/B2SProMotion.h"

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

namespace B2S {

class B2SServer;

class B2SProPhysics final
{
public:
   B2SProPhysics(B2STable& table, B2SProMotionSystem& motion);

   using Resolver = std::function<std::function<void()>(B2SRomIDType romIdType, int romId, float* target)>;
   void BindTriggers(const Resolver& resolver);
   void PollLaunchers(B2SServer* server);
   // Advances the physics to now (wall clock) and presents the balls
   void Advance(B2SServer* server);

   bool ActivateGravityDrop(B2SBulb& bulb);
   void DeactivateGravityDrop(B2SBulb& bulb);

   // B2SPictureBox physics pivot members
   static void SetPivotRotationTarget(ProBulbRuntime& rt, float targetAngle, int duration);
   static void AdvancePhysicsPivot(B2SBulb& pivot, double elapsed);
   static void ApplyPivotPhysicsAngularImpulse(B2SBulb& pivot, float deltaDegreesPerSecond);

private:
   struct Contact
   {
      float contactX = 0.f, contactY = 0.f, normalX = 0.f, normalY = 0.f, penetration = 0.f, distance = 0.f;
      int segmentIndex = 0;
   };
   struct Ball
   {
      B2SBulb* bulb = nullptr;
      string flipperName; // Lower case
      vector<string> impactPivotNames; // Lower case
      std::unordered_map<string, bool> activeSilhouetteContacts;
      ProRect bounds;
      float gravity = 0.f, flipperStrength = 1.f, boundaryBounce = 0.12f;
      vector<vector<ProPoint>> boundaryPaths;
      vector<vector<float>> boundarySegmentBounces;
      vector<ProRect> obstacles;
      vector<float> obstacleBounces;
      vector<ProSwitchZone> switchZones;
      vector<bool> switchZoneInside;
      bool hasLauncher = false;
      ProLauncher launcher;
      bool launcherArmed = true, launcherHolding = false, launcherExitedCapture = false;
      float authoredW = 0.f, authoredH = 0.f;
      ProPoint authoredCenter;
      double pendingSeconds = 0.0;
      bool active = true;
      ProPoint velocity;
      ProPoint center;
      float lastFlipperAngle = 0.f;
      bool hasLastFlipperAngle = false;
   };
   B2STable& m_table;
   B2SProMotionSystem& m_motion;
   B2SServer* m_server = nullptr;
   vector<Ball> m_balls;
   std::unordered_map<B2SBulb*, size_t> m_ballByBulb;
   std::unordered_map<string, B2SBulb*> m_pivotsByName; // PivotPicturesByName (lower case)
   vector<B2SBulb*> m_physicsPivots;

   struct LauncherRoute
   {
      vector<size_t> balls;
      float value = -1.f;
      float last = -1.f;
      std::function<void()> updater = []() { };
      uint64_t stamp = 0;
   };
   std::map<std::pair<int, int>, LauncherRoute> m_launchers; // (trigger type: 1 sol, 2 lamp, 3 B2SID, 5 switch, id)

   bool m_clockRunning = false;
   std::chrono::steady_clock::time_point m_lastAdvance;
   double m_pendingSeconds = 0.0;
   std::mt19937 m_launcherRandom { std::random_device {}() };
   std::mt19937 m_scatterRandom { std::random_device {}() };
   std::mt19937 m_strengthRandom { std::random_device {}() };

   B2SBulb* FindPivot(const string& lowerName) const;
   void StartClock();
   void Start(Ball& ball);
   void Launch(Ball& ball);
   void ActivateAtCurrentPosition(Ball& ball);
   static void DeactivateForMotionPath(Ball& ball);
   void AdvanceServerStep(Ball& ball);
   void AdvancePhysics(Ball& ball, double elapsed);
   void StepPhysics(Ball& ball, float elapsed, B2SBulb* flipper, float angularVelocity);
   void GetLauncherPose(const Ball& ball, ProPoint& origin, float& angle) const;
   static ProRect PivotBounds(const B2SBulb& pivot);
   void CheckLauncherCapture(Ball& ball, ProPoint& center);
   void CheckSwitchZones(Ball& ball, ProPoint previous, ProPoint center);
   void ResolveCircularObstacle(ProPoint& center, float ballRadius, ProPoint& velocity, const ProRect& obstacle, float bounce);
   void ResolvePivotObjectCollision(Ball& ball, ProPoint& center, ProPoint previous, float ballRadius, ProPoint& velocity, B2SBulb& pivot, const string& contactKey, float bounce);
   bool TryResolveSilhouette(Ball& ball, ProPoint& center, ProPoint previous, float ballRadius, ProPoint& velocity, B2SBulb& pivot, const ProRect& objectBounds, ProPoint hinge,
      ProPoint objectCenter, float objectRadius, bool isFreshContact);
   static bool TryFindContacts(float cx, float cy, float ballRadius, const vector<ProPoint>& contour, int maskW, int maskH, const ProRect& objectBounds, Contact contacts[2], int& count);
   static bool TryBuildContact(float cx, float cy, float ballRadius, const vector<ProPoint>& contour, int segmentIndex, float scaleX, float scaleY, const ProRect& objectBounds,
      bool centerInside, Contact& contact);
   void ResolveKnockdownPivot(ProPoint& center, ProPoint previous, float ballRadius, ProPoint& velocity, B2SBulb& pivot, const ProRect& objectBounds, float bounce);
   void ResolveStaticFloor(ProPoint& center, ProPoint previous, float radius, ProPoint& velocity, ProPoint from, ProPoint to, float bounce);
   bool TrySweptStaticFloor(ProPoint previous, ProPoint current, float radius, ProPoint from, ProPoint to, ProPoint& impactCenter, ProPoint& impactNormal) const;
   void ReflectBoundaryVelocity(ProPoint& velocity, float nx, float ny, float bounce, float incomingNormalSpeed);
   void ResolveFlipperCollision(Ball& ball, ProPoint& center, float ballRadius, ProPoint& velocity, B2SBulb& flipper, float angularVelocity);
};

// BuildPhysicsContour
vector<ProPoint> ProBuildPhysicsContour(const vector<uint8_t>& alpha, int maskW, int maskH, float pivotX, float pivotY);

}
