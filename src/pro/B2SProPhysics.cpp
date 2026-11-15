// license:GPLv3+

#include "B2SProPhysics.h"
#include "B2SProScore.h"
#include "B2SServer.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>

namespace B2S {

static constexpr double kPi = 3.14159265358979323846;
static constexpr float kPhysicsSubstepSeconds = 0.001f;
static constexpr float kShallowSurfaceSlopeLimit = 0.35f;
static constexpr float kShallowSurfaceRollMultiplier = 2.f;
static constexpr float kBoundaryScatterDegrees = 10.f;
static constexpr float kBoundaryScatterMinimumImpactSpeed = 150.f;
static constexpr int kMaxPhysicsStepsPerTick = 250;

static string LowerTrim(const string& s)
{
   size_t b = 0, e = s.size();
   while (b < e && std::isspace(static_cast<unsigned char>(s[b])))
      b++;
   while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])))
      e--;
   string r = s.substr(b, e - b);
   std::transform(r.begin(), r.end(), r.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
   return r;
}
static float NormalizeSwitchAngle(float angle)
{
   float normalized = std::fmod(angle, 360.f);
   if (normalized > 180.f)
      normalized -= 360.f;
   if (normalized <= -180.f)
      normalized += 360.f;
   return normalized;
}
static ProRect CurrentRect(const B2SBulb& b)
{
   if (b.m_pro && b.m_pro->rt.motion.active)
   {
      const ProMotion& m = b.m_pro->rt.motion;
      return { m.x, m.y, m.w, m.h };
   }
   return { static_cast<float>(b.m_locationX), static_cast<float>(b.m_locationY), static_cast<float>(b.m_width), static_cast<float>(b.m_height) };
}

vector<ProPoint> ProBuildPhysicsContour(const vector<uint8_t>& sourceAlpha, int maskWidth, int maskHeight, float normalizedPivotX, float normalizedPivotY)
{
   vector<ProPoint> contour;
   if (maskWidth <= 0 || maskHeight <= 0 || sourceAlpha.size() < static_cast<size_t>(maskWidth) * maskHeight)
      return contour;
   constexpr int angularBinCount = 720;
   vector<float> outerRadius(angularBinCount, 0.f);
   const float pivotX = static_cast<float>(std::clamp(static_cast<double>(normalizedPivotX), 0.0, 1.0)) * (maskWidth - 1);
   const float pivotY = static_cast<float>(std::clamp(static_cast<double>(normalizedPivotY), 0.0, 1.0)) * (maskHeight - 1);
   const double radiansToBin = angularBinCount / (kPi * 2.0);
   for (int y = 0; y < maskHeight; y++)
      for (int x = 0; x < maskWidth; x++)
      {
         if (sourceAlpha[static_cast<size_t>(y) * maskWidth + x] == 0)
            continue;
         const float dx = x - pivotX, dy = y - pivotY;
         double angle = std::atan2(dy, dx);
         if (angle < 0.0)
            angle += kPi * 2.0;
         int bin = static_cast<int>(std::floor(angle * radiansToBin));
         if (bin >= angularBinCount)
            bin = 0;
         const float radius = static_cast<float>(std::sqrt(dx * dx + dy * dy));
         if (radius > outerRadius[bin])
            outerRadius[bin] = radius;
      }
   for (int bin = 0; bin < angularBinCount; bin++)
   {
      if (outerRadius[bin] > 0.f)
         continue;
      int previousDistance = 1;
      while (previousDistance < angularBinCount && outerRadius[(bin - previousDistance + angularBinCount) % angularBinCount] <= 0.f)
         previousDistance++;
      int nextDistance = 1;
      while (nextDistance < angularBinCount && outerRadius[(bin + nextDistance) % angularBinCount] <= 0.f)
         nextDistance++;
      if (previousDistance < angularBinCount && nextDistance < angularBinCount)
      {
         const float previousRadius = outerRadius[(bin - previousDistance + angularBinCount) % angularBinCount];
         const float nextRadius = outerRadius[(bin + nextDistance) % angularBinCount];
         outerRadius[bin] = (previousRadius * nextDistance + nextRadius * previousDistance) / (previousDistance + nextDistance);
      }
   }
   contour.resize(angularBinCount);
   for (int bin = 0; bin < angularBinCount; bin++)
   {
      const double angle = bin / radiansToBin;
      contour[bin] = { static_cast<float>(pivotX + std::cos(angle) * outerRadius[bin]), static_cast<float>(pivotY + std::sin(angle) * outerRadius[bin]) };
   }
   return contour;
}

B2SProPhysics::B2SProPhysics(B2STable& table, B2SProMotionSystem& motion)
   : m_table(table)
   , m_motion(motion)
{
   for (auto* bulbs : { &table.m_backglassIlluminations, &table.m_dmdIlluminations })
      for (auto& bulbPtr : *bulbs)
      {
         B2SBulb* const bulb = bulbPtr.get();
         if (!bulb->m_pro || !bulb->m_pro->pivotAnimation)
            continue;
         // RegisterPivotPicture
         const string key = LowerTrim(bulb->m_pro->name);
         if (!key.empty())
            m_pivotsByName[key] = bulb;
         // ConfigurePhysicsCollisionOutline: binary alpha mask of the artwork (at most 512 x 512) and its contour
         ProImage image;
         if (bulb->m_width > 0 && bulb->m_height > 0 && ProReadTexture(bulb->m_image, image))
         {
            const int maskW = std::clamp(bulb->m_width, 1, 512), maskH = std::clamp(bulb->m_height, 1, 512);
            ProImage mask;
            mask.Allocate(maskW, maskH);
            ProDrawScaled(mask, image, 0.0, 0.0, maskW, maskH, true);
            vector<uint8_t> alpha(static_cast<size_t>(maskW) * maskH);
            bool hasSolid = false;
            for (size_t i = 0; i < alpha.size(); i++)
            {
               alpha[i] = mask.px[i * 4 + 3] > 0 ? 255 : 0;
               hasSolid |= alpha[i] != 0;
            }
            if (hasSolid)
            {
               ProBulbRuntime& rt = bulb->m_pro->rt;
               rt.collisionMaskW = maskW;
               rt.collisionMaskH = maskH;
               rt.collisionContour = ProBuildPhysicsContour(alpha, maskW, maskH, bulb->m_pro->pivotX, bulb->m_pro->pivotY);
            }
         }
      }

   // B2SData.RegisterPhysicsBall
   for (auto* bulbs : { &table.m_backglassIlluminations, &table.m_dmdIlluminations })
      for (auto& bulbPtr : *bulbs)
      {
         B2SBulb* const bulb = bulbPtr.get();
         if (!bulb->m_pro || !bulb->m_pro->physicsBall || bulb->m_pro->physicsBounds.IsEmpty())
            continue;
         const B2SProBulbData& pro = *bulb->m_pro;
         Ball ball;
         ball.bulb = bulb;
         ball.flipperName = pro.motionPathGravityDrop ? string() : LowerTrim(pro.physicsFlipperName);
         for (const string& name : pro.physicsPivotTargetNames)
         {
            const string key = LowerTrim(name);
            if (!key.empty() && std::find(ball.impactPivotNames.begin(), ball.impactPivotNames.end(), key) == ball.impactPivotNames.end())
               ball.impactPivotNames.push_back(key);
         }
         ball.bounds = pro.physicsBounds;
         ball.gravity = std::clamp(pro.physicsGravity, 0.f, 10000.f);
         ball.flipperStrength = std::clamp(pro.physicsFlipperStrength, 0.f, 5.f);
         ball.boundaryBounce = std::clamp(pro.physicsBoundaryBounce, 0.f, 1.f);
         ball.authoredW = bulb->m_width > 0 ? static_cast<float>(bulb->m_width) : 0.f;
         ball.authoredH = bulb->m_height > 0 ? static_cast<float>(bulb->m_height) : 0.f;
         ball.authoredCenter = { bulb->m_locationX + bulb->m_width / 2.f, bulb->m_locationY + bulb->m_height / 2.f };
         ball.center = ball.authoredCenter;
         for (size_t pathIndex = 0; pathIndex < pro.physicsBoundaryPaths.size(); pathIndex++)
         {
            const vector<ProPoint>& path = pro.physicsBoundaryPaths[pathIndex];
            if (path.size() < 2)
               continue;
            ball.boundaryPaths.push_back(path);
            vector<float> values;
            const vector<float>* saved = pathIndex < pro.physicsBoundarySegmentBounces.size() ? &pro.physicsBoundarySegmentBounces[pathIndex] : nullptr;
            for (size_t segment = 0; segment + 1 < path.size(); segment++)
               values.push_back(std::clamp(saved != nullptr && segment < saved->size() ? (*saved)[segment] : -1.f, -1.f, 3.f));
            ball.boundarySegmentBounces.push_back(std::move(values));
         }
         for (size_t index = 0; index < pro.physicsObstacles.size(); index++)
         {
            const ProRect& obstacle = pro.physicsObstacles[index];
            if (obstacle.w <= 0.f || obstacle.h <= 0.f)
               continue;
            ball.obstacles.push_back(obstacle);
            const float value = index < pro.physicsObstacleBounces.size() ? pro.physicsObstacleBounces[index] : -1.f;
            ball.obstacleBounces.push_back((std::isnan(value) || std::isinf(value) || value < 0.f) ? ball.boundaryBounce : std::min(3.f, value));
         }
         for (const ProSwitchZone& zone : pro.physicsSwitchZones)
            if (zone.bounds.w > 0.f && zone.bounds.h > 0.f && (zone.switchId > 0 || !LowerTrim(zone.soundName).empty()))
            {
               ProSwitchZone copy = zone;
               copy.switchId = std::max(0, zone.switchId);
               copy.angle = NormalizeSwitchAngle(zone.angle);
               copy.soundVolume = std::clamp(zone.soundVolume, 0.f, 2.f);
               ball.switchZones.push_back(copy);
               ball.switchZoneInside.push_back(false);
            }
         // A gravity trough ball has no launcher
         if (pro.hasPhysicsLauncher && !pro.motionPathGravityDrop)
         {
            ball.hasLauncher = true;
            ball.launcher = pro.physicsLauncher;
            ball.launcher.captureRadius = std::max(5.f, pro.physicsLauncher.captureRadius);
            ball.launcherHolding = true;
         }
         const size_t index = m_balls.size();
         m_ballByBulb[bulb] = index;
         if (ball.hasLauncher && ball.launcher.triggerId > 0)
         {
            const int type = ball.launcher.triggerType == 2 ? 2 : ball.launcher.triggerType == 5 ? 5 : ball.launcher.triggerType == 3 ? 3 : 1;
            m_launchers[{ type, ball.launcher.triggerId }].balls.push_back(index);
         }
         m_balls.push_back(std::move(ball));
      }

   // Pivots driven by the physics steps (PhysicsPivots of every ball: impact targets and the flipper)
   for (const Ball& ball : m_balls)
   {
      vector<string> names = ball.impactPivotNames;
      names.push_back(ball.flipperName);
      for (const string& name : names)
         if (B2SBulb* pivot = FindPivot(name); pivot != nullptr && std::find(m_physicsPivots.begin(), m_physicsPivots.end(), pivot) == m_physicsPivots.end())
         {
            m_physicsPivots.push_back(pivot);
            pivot->m_pro->rt.physicsPivot = true;
         }
   }

   for (Ball& ball : m_balls)
   {
      if (ball.bulb->m_pro->motionPathGravityDrop)
         DeactivateForMotionPath(ball);
      else
         Start(ball);
   }
}

B2SBulb* B2SProPhysics::FindPivot(const string& lowerName) const
{
   if (lowerName.empty())
      return nullptr;
   const auto it = m_pivotsByName.find(lowerName);
   return it == m_pivotsByName.end() ? nullptr : it->second;
}

void B2SProPhysics::BindTriggers(const Resolver& resolver)
{
   for (auto& [key, route] : m_launchers)
   {
      route.updater = []() { };
      if (key.first == 3)
         continue;
      const B2SRomIDType type = key.first == 2 ? B2SRomIDType::Lamp : key.first == 5 ? B2SRomIDType::Switch : B2SRomIDType::Solenoid;
      route.updater = resolver(type, key.second, &route.value);
   }
}

void B2SProPhysics::PollLaunchers(B2SServer* server)
{
   m_server = server;
   for (auto& [key, route] : m_launchers)
   {
      bool fire = false;
      if (key.first == 3)
      {
         // MyB2SSetData calls FirePhysicsLaunchers on every write; a non zero value launches
         float value = 0.f;
         uint64_t stamp = 0;
         if (server != nullptr && server->GetScriptedLampState(key.second, value, &stamp) && stamp != route.stamp)
         {
            route.stamp = stamp;
            fire = static_cast<int>(value) != 0;
         }
      }
      else
      {
         route.updater();
         if (route.value >= 0.f)
         {
            const bool isSwitch = key.first == 5;
            const bool on = isSwitch ? route.value > 0.f : route.value >= 0.5f;
            const bool known = route.last >= 0.f;
            const bool wasOn = known && (isSwitch ? route.last > 0.f : route.last >= 0.5f);
            route.last = route.value;
            fire = on && (!known || !wasOn);
         }
      }
      if (!fire)
         continue;
      for (size_t index : route.balls)
      {
         Advance(server); // SynchronizePhysics
         Launch(m_balls[index]);
      }
   }
}

void B2SProPhysics::StartClock()
{
   if (m_clockRunning)
      return;
   m_clockRunning = true;
   m_lastAdvance = std::chrono::steady_clock::now();
}

void B2SProPhysics::Start(Ball& ball)
{
   ball.active = true;
   ball.bulb->m_pro->rt.visible = true;
   ball.pendingSeconds = 0.0;
   StartClock();
}

void B2SProPhysics::Launch(Ball& ball)
{
   if (!ball.hasLauncher || !ball.launcherArmed)
      return;
   std::uniform_real_distribution<double> unit(0.0, 1.0);
   const float randomAngleOffset = static_cast<float>((unit(m_launcherRandom) * 2.0 - 1.0) * ball.launcher.randomAngle);
   const float randomStrengthFactor = 1.f + static_cast<float>((unit(m_launcherRandom) * 2.0 - 1.0) * ball.launcher.randomStrength / 100.0);
   ProPoint origin;
   float angle = 0.f;
   GetLauncherPose(ball, origin, angle);
   const double radians = (angle + randomAngleOffset) * kPi / 180.0;
   const float launchStrength = std::max(0.f, ball.launcher.strength * randomStrengthFactor);
   ball.center = origin;
   m_motion.SetMotionPathPosition(*ball.bulb, origin);
   ball.velocity = { static_cast<float>(std::cos(radians)) * launchStrength, static_cast<float>(std::sin(radians)) * launchStrength };
   ball.launcherArmed = false;
   ball.launcherHolding = false;
   ball.launcherExitedCapture = false;
   ball.activeSilhouetteContacts.clear();
   std::fill(ball.switchZoneInside.begin(), ball.switchZoneInside.end(), false);
}

void B2SProPhysics::ActivateAtCurrentPosition(Ball& ball)
{
   ball.center = B2SProMotionSystem::Center(*ball.bulb);
   ball.velocity = {};
   ball.pendingSeconds = 0.0;
   ball.launcherHolding = false;
   ball.launcherArmed = false;
   ball.hasLastFlipperAngle = false;
   ball.activeSilhouetteContacts.clear();
   ball.active = true;
   ball.bulb->m_pro->rt.visible = true;
}

void B2SProPhysics::DeactivateForMotionPath(Ball& ball)
{
   ball.active = false;
   ball.velocity = {};
   ball.pendingSeconds = 0.0;
   ball.hasLastFlipperAngle = false;
   ball.launcherHolding = false;
   ball.launcherArmed = false;
   ball.activeSilhouetteContacts.clear();
}

bool B2SProPhysics::ActivateGravityDrop(B2SBulb& bulb)
{
   const auto it = m_ballByBulb.find(&bulb);
   if (it == m_ballByBulb.end())
      return false;
   ActivateAtCurrentPosition(m_balls[it->second]);
   StartClock();
   return true;
}

void B2SProPhysics::DeactivateGravityDrop(B2SBulb& bulb)
{
   if (const auto it = m_ballByBulb.find(&bulb); it != m_ballByBulb.end())
      DeactivateForMotionPath(m_balls[it->second]);
}

void B2SProPhysics::Advance(B2SServer* server)
{
   if (server != nullptr)
      m_server = server;
   if (!m_clockRunning)
      return;
   const auto now = std::chrono::steady_clock::now();
   double elapsed = std::chrono::duration<double>(now - m_lastAdvance).count();
   m_lastAdvance = now;
   if (elapsed <= 0.0 || std::isnan(elapsed) || std::isinf(elapsed))
      return;
   elapsed = std::min(elapsed, 1.0); // The renderer clamps its frame time the same way
   // B2SData.AdvanceServerPhysics: pivots then balls, in 1 ms steps, then present the completed frame
   m_pendingSeconds += elapsed;
   const int steps = static_cast<int>(std::floor((m_pendingSeconds + 0.000000001) / 0.001));
   for (int index = 0; index < steps; index++)
   {
      for (B2SBulb* pivot : m_physicsPivots)
         AdvancePhysicsPivot(*pivot, 0.001);
      for (Ball& ball : m_balls)
         AdvanceServerStep(ball);
   }
   m_pendingSeconds = std::max(0.0, m_pendingSeconds - steps * 0.001);
   for (Ball& ball : m_balls)
      if (ball.active)
         m_motion.SetMotionPathPosition(*ball.bulb, ball.center, true);
}

void B2SProPhysics::AdvanceServerStep(Ball& ball)
{
   if (!ball.active)
      return;
   const ProRect rect = CurrentRect(*ball.bulb);
   if (rect.w <= 0.f || rect.h <= 0.f)
      return;
   if (!ball.hasLastFlipperAngle)
      if (B2SBulb* pivot = FindPivot(ball.flipperName))
      {
         ball.lastFlipperAngle = pivot->m_pro->rt.physicsPreviousAngle;
         ball.hasLastFlipperAngle = true;
      }
   AdvancePhysics(ball, 0.001);
}

void B2SProPhysics::AdvancePhysics(Ball& ball, double elapsed)
{
   if (!ball.active || elapsed <= 0.0 || std::isnan(elapsed) || std::isinf(elapsed))
      return;
   ball.pendingSeconds += elapsed;
   B2SBulb* flipper = FindPivot(ball.flipperName);
   float angularVelocity = 0.f;
   if (flipper != nullptr && std::find(ball.impactPivotNames.begin(), ball.impactPivotNames.end(), ball.flipperName) != ball.impactPivotNames.end())
   {
      if (ball.hasLastFlipperAngle)
         angularVelocity = static_cast<float>((flipper->m_pro->rt.angle - ball.lastFlipperAngle) * kPi / 180.0 / elapsed);
      ball.lastFlipperAngle = flipper->m_pro->rt.angle;
      ball.hasLastFlipperAngle = true;
   }
   const int steps = static_cast<int>(std::min(static_cast<double>(kMaxPhysicsStepsPerTick), std::floor((ball.pendingSeconds + 0.000000001) / 0.001)));
   for (int index = 0; index < steps; index++)
      StepPhysics(ball, kPhysicsSubstepSeconds, flipper, angularVelocity);
   ball.pendingSeconds = std::max(0.0, ball.pendingSeconds - steps * 0.001);
}

void B2SProPhysics::StepPhysics(Ball& ball, float elapsed, B2SBulb* flipper, float angularVelocity)
{
   ProPoint center = ball.center;
   if (ball.launcherHolding && ball.hasLauncher)
   {
      ProPoint hold;
      float angle = 0.f;
      GetLauncherPose(ball, hold, angle);
      ball.center = hold;
      ball.velocity = {};
      return;
   }
   const ProRect& b = ball.bounds;
   const ProPoint previous = center;
   ProPoint& velocity = ball.velocity;
   velocity.y += ball.gravity * elapsed;
   center.x += velocity.x * elapsed;
   center.y += velocity.y * elapsed;
   const float radius = std::max(2.f, std::min(ball.authoredW, ball.authoredH) * 0.43f);
   if (center.x - radius < b.x)
   {
      center.x = b.x + radius;
      velocity.x = std::abs(velocity.x) * 0.55f;
   }
   if (center.x + radius > b.x + b.w)
   {
      center.x = b.x + b.w - radius;
      velocity.x = -std::abs(velocity.x) * 0.55f;
   }
   if (center.y - radius < b.y)
   {
      center.y = b.y + radius;
      velocity.y = std::abs(velocity.y) * 0.55f;
   }
   if (center.y + radius > b.y + b.h)
   {
      center.y = b.y + b.h - radius;
      velocity.y = -std::abs(velocity.y) * 0.55f;
   }
   if (flipper != nullptr)
   {
      // A physics impact pivot stays a collision object even when the launcher follows it
      if (flipper->m_pro->pivotPhysicsEnabled)
         ResolvePivotObjectCollision(ball, center, previous, radius, velocity, *flipper, ball.flipperName, ball.boundaryBounce);
      else if (!ball.hasLauncher || !ball.launcher.followPivot)
         ResolveFlipperCollision(ball, center, radius, velocity, *flipper, angularVelocity);
   }
   for (const string& targetName : ball.impactPivotNames)
   {
      if (targetName == ball.flipperName)
         continue;
      B2SBulb* target = FindPivot(targetName);
      if (target != nullptr && target->m_pro->pivotPhysicsEnabled)
         ResolvePivotObjectCollision(ball, center, previous, radius, velocity, *target, targetName, ball.boundaryBounce);
   }
   for (size_t pathIndex = 0; pathIndex < ball.boundaryPaths.size(); pathIndex++)
   {
      const vector<ProPoint>& path = ball.boundaryPaths[pathIndex];
      for (size_t segment = 0; segment + 1 < path.size(); segment++)
      {
         const ProPoint from = path[segment], to = path[segment + 1];
         float segmentBounce = ball.boundaryBounce;
         if (pathIndex < ball.boundarySegmentBounces.size() && segment < ball.boundarySegmentBounces[pathIndex].size() && ball.boundarySegmentBounces[pathIndex][segment] >= 0.f)
            segmentBounce = ball.boundarySegmentBounces[pathIndex][segment];
         const ProPoint beforeCenter = center, beforeVelocity = velocity;
         ResolveStaticFloor(center, previous, radius, velocity, from, to, segmentBounce);
         const bool resolved = std::abs(center.x - beforeCenter.x) > 0.0001f || std::abs(center.y - beforeCenter.y) > 0.0001f || std::abs(velocity.x - beforeVelocity.x) > 0.0001f
            || std::abs(velocity.y - beforeVelocity.y) > 0.0001f;
         if (resolved)
         {
            // ApplyShallowSurfaceRollingBoost
            const float sx = to.x - from.x, sy = to.y - from.y;
            const float length = static_cast<float>(std::sqrt(sx * sx + sy * sy));
            if (length > 0.01f)
            {
               const float tx = sx / length, ty = sy / length;
               const float slope = std::abs(ty);
               if (slope > 0.001f && slope <= kShallowSurfaceSlopeLimit)
               {
                  const float extra = ball.gravity * ty * (kShallowSurfaceRollMultiplier - 1.f);
                  velocity.x += tx * extra * elapsed;
                  velocity.y += ty * extra * elapsed;
               }
            }
         }
      }
   }
   for (size_t index = 0; index < ball.obstacles.size(); index++)
      ResolveCircularObstacle(center, radius, velocity, ball.obstacles[index], ball.obstacleBounces[index]);
   CheckLauncherCapture(ball, center);
   CheckSwitchZones(ball, previous, center);
   ball.center = center;
}

ProRect B2SProPhysics::PivotBounds(const B2SBulb& pivot) { return CurrentRect(pivot); }

void B2SProPhysics::GetLauncherPose(const Ball& ball, ProPoint& origin, float& angle) const
{
   origin = ball.launcher.origin;
   angle = ball.launcher.angle;
   if (!ball.launcher.followPivot)
      return;
   const B2SBulb* pivot = FindPivot(ball.flipperName);
   if (pivot == nullptr)
      return;
   const ProBulbRuntime& rt = pivot->m_pro->rt;
   if (rt.automaticOscillation)
      origin = ball.authoredCenter;
   const ProRect bounds = PivotBounds(*pivot);
   const ProPoint hinge { bounds.x + bounds.w * pivot->m_pro->pivotX, bounds.y + bounds.h * pivot->m_pro->pivotY };
   const float restAngle = rt.automaticOscillation ? 0.f : rt.downAngle;
   const float delta = rt.angle - restAngle;
   const double radians = delta * kPi / 180.0;
   const float dx = origin.x - hinge.x, dy = origin.y - hinge.y;
   origin = { hinge.x + static_cast<float>(dx * std::cos(radians) - dy * std::sin(radians)), hinge.y + static_cast<float>(dx * std::sin(radians) + dy * std::cos(radians)) };
   angle += delta;
}

void B2SProPhysics::CheckLauncherCapture(Ball& ball, ProPoint& center)
{
   if (!ball.hasLauncher || ball.launcherArmed)
      return;
   ProPoint origin;
   float angle = 0.f;
   GetLauncherPose(ball, origin, angle);
   const float radius = ball.launcher.captureRadius;
   const float dx = center.x - origin.x, dy = center.y - origin.y;
   const bool inside = dx * dx + dy * dy <= radius * radius;
   if (!ball.launcherExitedCapture)
      ball.launcherExitedCapture = !inside;
   else if (inside)
   {
      center = origin;
      ball.velocity = {};
      ball.launcherArmed = true;
      ball.launcherHolding = true;
   }
}

static ProPoint SwitchZoneLocalPoint(ProPoint point, const ProSwitchZone& zone)
{
   const float angle = NormalizeSwitchAngle(zone.angle);
   if (std::abs(angle) < 0.001f)
      return point;
   const float cx = zone.bounds.x + zone.bounds.w / 2.f, cy = zone.bounds.y + zone.bounds.h / 2.f;
   const double radians = -angle * kPi / 180.0;
   const double c = std::cos(radians), s = std::sin(radians);
   const double x = point.x - cx, y = point.y - cy;
   return { static_cast<float>(cx + x * c - y * s), static_cast<float>(cy + x * s + y * c) };
}
static bool ZoneContains(const ProRect& r, ProPoint p) { return p.x >= r.x && p.x < r.x + r.w && p.y >= r.y && p.y < r.y + r.h; }
static bool ClipSwitchSegment(double direction, double distance, double& first, double& last)
{
   if (std::abs(direction) < 0.0000001)
      return distance >= 0.0;
   const double ratio = distance / direction;
   if (direction < 0.0)
   {
      if (ratio > last)
         return false;
      if (ratio > first)
         first = ratio;
   }
   else
   {
      if (ratio < first)
         return false;
      if (ratio < last)
         last = ratio;
   }
   return true;
}

void B2SProPhysics::CheckSwitchZones(Ball& ball, ProPoint previous, ProPoint center)
{
   for (size_t index = 0; index < ball.switchZones.size(); index++)
   {
      const ProSwitchZone& zone = ball.switchZones[index];
      const bool inside = ZoneContains(zone.bounds, SwitchZoneLocalPoint(center, zone));
      bool crossed = false;
      if (!inside)
      {
         const ProPoint from = SwitchZoneLocalPoint(previous, zone), to = SwitchZoneLocalPoint(center, zone);
         if (ZoneContains(zone.bounds, from) || ZoneContains(zone.bounds, to))
            crossed = true;
         else
         {
            const double dx = to.x - from.x, dy = to.y - from.y;
            double first = 0.0, last = 1.0;
            crossed = ClipSwitchSegment(-dx, from.x - zone.bounds.x, first, last) && ClipSwitchSegment(dx, zone.bounds.x + zone.bounds.w - from.x, first, last)
               && ClipSwitchSegment(-dy, from.y - zone.bounds.y, first, last) && ClipSwitchSegment(dy, zone.bounds.y + zone.bounds.h - from.y, first, last);
         }
      }
      if (!ball.switchZoneInside[index] && (inside || crossed) && m_server != nullptr)
      {
         if (zone.switchId > 0)
            m_server->PulseSwitch(zone.switchId); // B2SAnimation.SetSwitch
         if (!LowerTrim(zone.soundName).empty())
            m_server->B2SPlayPhysicsSound(zone.soundName, zone.soundVolume);
      }
      ball.switchZoneInside[index] = inside;
   }
}

void B2SProPhysics::ResolveCircularObstacle(ProPoint& center, float ballRadius, ProPoint& velocity, const ProRect& obstacle, float bounce)
{
   const float ocx = obstacle.x + obstacle.w / 2.f, ocy = obstacle.y + obstacle.h / 2.f;
   const float obstacleRadius = std::max(1.f, (obstacle.w + obstacle.h) / 4.f);
   const float dx = center.x - ocx, dy = center.y - ocy;
   const float minimumDistance = ballRadius + obstacleRadius;
   const float distanceSquared = dx * dx + dy * dy;
   if (distanceSquared >= minimumDistance * minimumDistance)
      return;
   float distance = static_cast<float>(std::sqrt(distanceSquared));
   float nx, ny;
   if (distance < 0.001f)
   {
      nx = 0.f;
      ny = -1.f;
      distance = 0.f;
   }
   else
   {
      nx = dx / distance;
      ny = dy / distance;
   }
   center.x += nx * (minimumDistance - distance);
   center.y += ny * (minimumDistance - distance);
   const float normalSpeed = velocity.x * nx + velocity.y * ny;
   if (normalSpeed < 0.f)
   {
      velocity.x -= (1.f + bounce) * normalSpeed * nx;
      velocity.y -= (1.f + bounce) * normalSpeed * ny;
   }
}

void B2SProPhysics::ResolvePivotObjectCollision(Ball& ball, ProPoint& center, ProPoint previous, float ballRadius, ProPoint& velocity, B2SBulb& pivot, const string& contactKey, float bounce)
{
   const ProRect objectBounds = PivotBounds(pivot);
   if (objectBounds.w <= 0.f || objectBounds.h <= 0.f)
      return;
   const B2SProBulbData& pp = *pivot.m_pro;
   ProBulbRuntime& prt = pivot.m_pro->rt;
   if (pp.pivotPhysicsResponseMode == 1 || pp.pivotPhysicsResponseMode == 2)
   {
      ResolveKnockdownPivot(center, previous, ballRadius, velocity, pivot, objectBounds, bounce);
      return;
   }
   const ProPoint hinge { objectBounds.x + objectBounds.w * pp.pivotX, objectBounds.y + objectBounds.h * pp.pivotY };
   const ProPoint sourceCenter { objectBounds.x + objectBounds.w / 2.f, objectBounds.y + objectBounds.h / 2.f };
   const double radians = prt.angle * kPi / 180.0;
   const double sourceX = sourceCenter.x - hinge.x, sourceY = sourceCenter.y - hinge.y;
   const ProPoint objectCenter { static_cast<float>(hinge.x + sourceX * std::cos(radians) - sourceY * std::sin(radians)),
      static_cast<float>(hinge.y + sourceX * std::sin(radians) + sourceY * std::cos(radians)) };
   const float objectRadius = std::max(3.f, std::min(objectBounds.w, objectBounds.h) / 2.f);
   float nx, ny, contactX, contactY;
   const bool useSilhouette = prt.collisionContour.size() >= 3 && prt.collisionMaskW > 0 && prt.collisionMaskH > 0;
   if (useSilhouette)
   {
      const bool isFreshContact = contactKey.empty() || ball.activeSilhouetteContacts.find(contactKey) == ball.activeSilhouetteContacts.end();
      if (TryResolveSilhouette(ball, center, previous, ballRadius, velocity, pivot, objectBounds, hinge, objectCenter, objectRadius, isFreshContact))
      {
         if (!contactKey.empty())
            ball.activeSilhouetteContacts[contactKey] = true;
      }
      else if (!contactKey.empty())
         ball.activeSilhouetteContacts.erase(contactKey);
      return;
   }
   float dx = center.x - objectCenter.x, dy = center.y - objectCenter.y;
   const float minimumDistance = ballRadius + objectRadius;
   float distanceSquared = dx * dx + dy * dy;
   if (distanceSquared >= minimumDistance * minimumDistance)
   {
      // Sweep the full movement against the collision circle
      const float mx = center.x - previous.x, my = center.y - previous.y;
      const float motionLengthSquared = mx * mx + my * my;
      if (motionLengthSquared <= 0.000001f)
         return;
      const float rx = previous.x - objectCenter.x, ry = previous.y - objectCenter.y;
      const float linear = 2.f * (rx * mx + ry * my);
      const float constant = rx * rx + ry * ry - minimumDistance * minimumDistance;
      const double discriminant = static_cast<double>(linear) * linear - 4.0 * static_cast<double>(motionLengthSquared) * constant;
      if (discriminant < 0.0)
         return;
      const float hitTime = static_cast<float>((-static_cast<double>(linear) - std::sqrt(discriminant)) / (2.0 * motionLengthSquared));
      if (hitTime < 0.f || hitTime > 1.f)
         return;
      center.x = previous.x + mx * hitTime;
      center.y = previous.y + my * hitTime;
      dx = center.x - objectCenter.x;
      dy = center.y - objectCenter.y;
      distanceSquared = dx * dx + dy * dy;
   }
   float distance = static_cast<float>(std::sqrt(distanceSquared));
   if (distance < 0.001f)
   {
      const float speed = static_cast<float>(std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y));
      if (speed > 0.001f)
      {
         nx = -velocity.x / speed;
         ny = -velocity.y / speed;
      }
      else
      {
         nx = 0.f;
         ny = -1.f;
      }
      distance = 0.f;
   }
   else
   {
      nx = dx / distance;
      ny = dy / distance;
   }
   center.x += nx * (minimumDistance - distance);
   center.y += ny * (minimumDistance - distance);
   contactX = objectCenter.x + nx * objectRadius;
   contactY = objectCenter.y + ny * objectRadius;

   const float armX = contactX - hinge.x, armY = contactY - hinge.y;
   const float angularRadians = prt.physicsAngularVelocity * static_cast<float>(kPi / 180.0);
   const float surfaceX = -angularRadians * armY, surfaceY = angularRadians * armX;
   const float relativeX = velocity.x - surfaceX, relativeY = velocity.y - surfaceY;
   const float incomingNormalSpeed = relativeX * nx + relativeY * ny;
   if (incomingNormalSpeed >= 0.f)
      return;
   const float weight = std::max(0.1f, pp.pivotPhysicsWeight);
   const float cox = objectCenter.x - hinge.x, coy = objectCenter.y - hinge.y;
   const float inertia = weight * std::max(1.f, 0.5f * objectRadius * objectRadius + cox * cox + coy * coy);
   const float inverseInertia = 1.f / inertia;
   const float armCrossNormal = armX * ny - armY * nx;
   const float normalDenominator = 1.f + armCrossNormal * armCrossNormal * inverseInertia;
   const float normalImpulse = -(1.f + std::clamp(bounce, 0.f, 1.f)) * incomingNormalSpeed / normalDenominator;
   velocity.x += normalImpulse * nx;
   velocity.y += normalImpulse * ny;
   const float tx = -ny, ty = nx;
   const float relativeTangentSpeed = relativeX * tx + relativeY * ty;
   const float armCrossTangent = armX * ty - armY * tx;
   const float tangentDenominator = 1.f + armCrossTangent * armCrossTangent * inverseInertia;
   float tangentImpulse = -relativeTangentSpeed / tangentDenominator;
   const float maximumTangentImpulse = std::abs(normalImpulse) * std::clamp(pp.pivotPhysicsGrip, 0.f, 1.f);
   tangentImpulse = std::clamp(tangentImpulse, -maximumTangentImpulse, maximumTangentImpulse);
   velocity.x += tangentImpulse * tx;
   velocity.y += tangentImpulse * ty;
   const float objectTorque = -normalImpulse * armCrossNormal - tangentImpulse * armCrossTangent;
   const float deltaDegreesPerSecond = objectTorque * inverseInertia * static_cast<float>(180.0 / kPi) * std::max(0.f, ball.flipperStrength);
   ApplyPivotPhysicsAngularImpulse(pivot, deltaDegreesPerSecond);
}

bool B2SProPhysics::TryResolveSilhouette(Ball& ball, ProPoint& center, ProPoint previous, float ballRadius, ProPoint& velocity, B2SBulb& pivot, const ProRect& objectBounds,
   ProPoint hinge, ProPoint objectCenter, float objectRadius, bool isFreshContact)
{
   (void)ball;
   ProBulbRuntime& prt = pivot.m_pro->rt;
   const B2SProBulbData& pp = *pivot.m_pro;
   const int maskW = prt.collisionMaskW, maskH = prt.collisionMaskH;
   if (maskW <= 0 || maskH <= 0)
      return false;
   const double radians = prt.angle * kPi / 180.0;
   const float cosine = static_cast<float>(std::cos(radians)), sine = static_cast<float>(std::sin(radians));
   const float pox = previous.x - hinge.x, poy = previous.y - hinge.y;
   const float cox = center.x - hinge.x, coy = center.y - hinge.y;
   const float previousLocalX = hinge.x + pox * cosine + poy * sine, previousLocalY = hinge.y - pox * sine + poy * cosine;
   const float currentLocalX = hinge.x + cox * cosine + coy * sine, currentLocalY = hinge.y - cox * sine + coy * cosine;
   const float scx = objectBounds.x + objectBounds.w / 2.f, scy = objectBounds.y + objectBounds.h / 2.f;
   const float sweepX = currentLocalX - previousLocalX, sweepY = currentLocalY - previousLocalY;
   const float sweepLengthSquared = sweepX * sweepX + sweepY * sweepY;
   float closestTime = 0.f;
   if (sweepLengthSquared > 0.000001f)
      closestTime = std::clamp(((scx - previousLocalX) * sweepX + (scy - previousLocalY) * sweepY) / sweepLengthSquared, 0.f, 1.f);
   const float broadX = previousLocalX + sweepX * closestTime - scx, broadY = previousLocalY + sweepY * closestTime - scy;
   const float broadRadius = ballRadius + static_cast<float>(std::sqrt(objectBounds.w * objectBounds.w + objectBounds.h * objectBounds.h)) / 2.f;
   if (broadX * broadX + broadY * broadY > broadRadius * broadRadius)
      return false;
   const float sweepLength = static_cast<float>(std::sqrt(sweepLengthSquared));
   const float sampleSpacing = std::clamp(ballRadius * 0.25f, 0.5f, 2.f);
   const int sampleCount = std::max(1, static_cast<int>(std::ceil(sweepLength / sampleSpacing)));
   Contact contacts[2];
   int contactCount = 0;
   float hitLocalX = 0.f, hitLocalY = 0.f;
   // Resolve a present overlap; a previous touch that is now clear is a separation, not a new hit
   if (TryFindContacts(currentLocalX, currentLocalY, ballRadius, prt.collisionContour, maskW, maskH, objectBounds, contacts, contactCount))
   {
      hitLocalX = currentLocalX;
      hitLocalY = currentLocalY;
   }
   else
   {
      Contact previousContacts[2];
      int previousCount = 0;
      if (TryFindContacts(previousLocalX, previousLocalY, ballRadius, prt.collisionContour, maskW, maskH, objectBounds, previousContacts, previousCount))
         return false;
      for (int sampleIndex = 1; sampleIndex < sampleCount; sampleIndex++)
      {
         const float fraction = static_cast<float>(sampleIndex) / static_cast<float>(sampleCount);
         const float sx = previousLocalX + sweepX * fraction, sy = previousLocalY + sweepY * fraction;
         if (TryFindContacts(sx, sy, ballRadius, prt.collisionContour, maskW, maskH, objectBounds, contacts, contactCount))
         {
            hitLocalX = sx;
            hitLocalY = sy;
            break;
         }
      }
   }
   if (contactCount <= 0)
      return false;

   float correctionX = 0.f, correctionY = 0.f;
   const float separationMargin = std::max(objectBounds.w / std::max(1, maskW - 1), objectBounds.h / std::max(1, maskH - 1));
   for (int iteration = 0; iteration <= 5; iteration++)
      for (int c = 0; c < contactCount; c++)
      {
         const float along = correctionX * contacts[c].normalX + correctionY * contacts[c].normalY;
         const float remaining = contacts[c].penetration + separationMargin - along;
         if (remaining > 0.f)
         {
            correctionX += remaining * contacts[c].normalX;
            correctionY += remaining * contacts[c].normalY;
         }
      }
   const float sepX = hitLocalX + correctionX - hinge.x, sepY = hitLocalY + correctionY - hinge.y;
   center.x = hinge.x + sepX * cosine - sepY * sine;
   center.y = hinge.y + sepX * sine + sepY * cosine;

   const float hox = hitLocalX - hinge.x, hoy = hitLocalY - hinge.y;
   const float hitArmX = hox * cosine - hoy * sine, hitArmY = hox * sine + hoy * cosine;
   const float incomingOmega = prt.physicsAngularVelocity * static_cast<float>(kPi / 180.0);
   const float isx = -incomingOmega * hitArmY, isy = incomingOmega * hitArmX;
   const float incomingAngularMomentum = hitArmX * (velocity.y - isy) - hitArmY * (velocity.x - isx);
   int desiredSpinSign = 0;
   if (isFreshContact && std::abs(incomingAngularMomentum) > 100.f)
      desiredSpinSign = (incomingAngularMomentum > 0.f) - (incomingAngularMomentum < 0.f);

   float worldNormalX[2], worldNormalY[2], armX[2], armY[2];
   for (int c = 0; c < contactCount; c++)
   {
      worldNormalX[c] = contacts[c].normalX * cosine - contacts[c].normalY * sine;
      worldNormalY[c] = contacts[c].normalX * sine + contacts[c].normalY * cosine;
      const float cx = contacts[c].contactX - hinge.x, cy = contacts[c].contactY - hinge.y;
      const float wx = hinge.x + cx * cosine - cy * sine, wy = hinge.y + cx * sine + cy * cosine;
      armX[c] = wx - hinge.x;
      armY[c] = wy - hinge.y;
   }
   const float weight = std::max(0.1f, pp.pivotPhysicsWeight);
   const float ocx = objectCenter.x - hinge.x, ocy = objectCenter.y - hinge.y;
   const float inertia = weight * std::max(1.f, 0.5f * objectRadius * objectRadius + ocx * ocx + ocy * ocy);
   const float inverseInertia = 1.f / inertia;
   const float startingOmega = prt.physicsAngularVelocity * static_cast<float>(kPi / 180.0);
   float omega = startingOmega;
   float accumulated[2] = { 0.f, 0.f }, target[2] = { 0.f, 0.f };
   constexpr float pivotRestitution = 0.55f;
   for (int c = 0; c < contactCount; c++)
   {
      const float sx = -omega * armY[c], sy = omega * armX[c];
      const float incoming = (velocity.x - sx) * worldNormalX[c] + (velocity.y - sy) * worldNormalY[c];
      target[c] = (isFreshContact && incoming < 0.f) ? -pivotRestitution * incoming : 0.f;
   }
   // Projected Gauss-Seidel: equal and opposite impulses for the ball and the pivot
   for (int iteration = 0; iteration <= 5; iteration++)
      for (int c = 0; c < contactCount; c++)
      {
         const float sx = -omega * armY[c], sy = omega * armX[c];
         const float current = (velocity.x - sx) * worldNormalX[c] + (velocity.y - sy) * worldNormalY[c];
         const float armCrossNormal = armX[c] * worldNormalY[c] - armY[c] * worldNormalX[c];
         const float effectiveMass = 1.f + armCrossNormal * armCrossNormal * inverseInertia;
         float impulseChange = (target[c] - current) / effectiveMass;
         const float newAccumulated = std::max(0.f, accumulated[c] + impulseChange);
         impulseChange = newAccumulated - accumulated[c];
         accumulated[c] = newAccumulated;
         if (std::abs(impulseChange) <= 0.000001f)
            continue;
         const float ix = impulseChange * worldNormalX[c], iy = impulseChange * worldNormalY[c];
         velocity.x += ix;
         velocity.y += iy;
         omega += -inverseInertia * (armX[c] * iy - armY[c] * ix);
      }
   const float grip = std::clamp(pp.pivotPhysicsGrip, 0.f, 1.f);
   if (isFreshContact)
      for (int c = 0; c < contactCount; c++)
      {
         if (accumulated[c] <= 0.f)
            continue;
         const float tx = -worldNormalY[c], ty = worldNormalX[c];
         const float sx = -omega * armY[c], sy = omega * armX[c];
         const float relativeTangent = (velocity.x - sx) * tx + (velocity.y - sy) * ty;
         const float armCrossTangent = armX[c] * ty - armY[c] * tx;
         const float tangentMass = 1.f + armCrossTangent * armCrossTangent * inverseInertia;
         float tangentImpulse = -relativeTangent / tangentMass;
         const float maximum = grip * accumulated[c];
         tangentImpulse = std::clamp(tangentImpulse, -maximum, maximum);
         const float tix = tangentImpulse * tx, tiy = tangentImpulse * ty;
         velocity.x += tix;
         velocity.y += tiy;
         omega += -inverseInertia * (armX[c] * tiy - armY[c] * tix);
      }
   const float solvedAngularChange = omega - startingOmega;
   if (isFreshContact && desiredSpinSign != 0 && solvedAngularChange * desiredSpinSign < 0.f)
   {
      omega = startingOmega + std::abs(solvedAngularChange) * desiredSpinSign;
      for (int c = 0; c < contactCount; c++)
      {
         const float sx = -omega * armY[c], sy = omega * armX[c];
         const float relativeNormal = (velocity.x - sx) * worldNormalX[c] + (velocity.y - sy) * worldNormalY[c];
         if (relativeNormal < 0.f)
         {
            velocity.x -= relativeNormal * worldNormalX[c];
            velocity.y -= relativeNormal * worldNormalY[c];
         }
      }
   }
   const float deltaDegreesPerSecond = (omega - startingOmega) * static_cast<float>(180.0 / kPi);
   if (std::abs(deltaDegreesPerSecond) > 0.000001f)
      ApplyPivotPhysicsAngularImpulse(pivot, deltaDegreesPerSecond);
   return true;
}

bool B2SProPhysics::TryFindContacts(float cx, float cy, float ballRadius, const vector<ProPoint>& contour, int maskW, int maskH, const ProRect& objectBounds, Contact contacts[2], int& count)
{
   count = 0;
   if (contour.size() < 3 || maskW <= 1 || maskH <= 1)
      return false;
   const float scaleX = objectBounds.w / (maskW - 1), scaleY = objectBounds.h / (maskH - 1);
   bool centerInside = false;
   const size_t n = contour.size();
   for (size_t index = 0; index < n; index++)
   {
      const size_t next = (index + 1) % n;
      const float sx = objectBounds.x + contour[index].x * scaleX, sy = objectBounds.y + contour[index].y * scaleY;
      const float ex = objectBounds.x + contour[next].x * scaleX, ey = objectBounds.y + contour[next].y * scaleY;
      if ((sy > cy) != (ey > cy))
      {
         const float crossingX = sx + (cy - sy) * (ex - sx) / (ey - sy);
         if (cx < crossingX)
            centerInside = !centerInside;
      }
   }
   bool primaryFound = false;
   Contact primary;
   for (size_t index = 0; index < n; index++)
   {
      Contact candidate;
      if (TryBuildContact(cx, cy, ballRadius, contour, static_cast<int>(index), scaleX, scaleY, objectBounds, centerInside, candidate))
         if (!primaryFound || candidate.distance < primary.distance)
         {
            primary = candidate;
            primaryFound = true;
         }
   }
   if (!primaryFound)
      return false;
   contacts[0] = primary;
   count = 1;
   bool secondaryFound = false;
   Contact secondary;
   for (size_t index = 0; index < n; index++)
   {
      Contact candidate;
      if (!TryBuildContact(cx, cy, ballRadius, contour, static_cast<int>(index), scaleX, scaleY, objectBounds, centerInside, candidate))
         continue;
      if (candidate.segmentIndex == primary.segmentIndex)
         continue;
      if (primary.normalX * candidate.normalX + primary.normalY * candidate.normalY >= 0.98f)
         continue;
      const float dx = candidate.contactX - primary.contactX, dy = candidate.contactY - primary.contactY;
      if (dx * dx + dy * dy <= 0.25f)
         continue;
      if (!secondaryFound || candidate.distance < secondary.distance)
      {
         secondary = candidate;
         secondaryFound = true;
      }
   }
   if (secondaryFound)
   {
      contacts[1] = secondary;
      count = 2;
   }
   return true;
}

bool B2SProPhysics::TryBuildContact(float cx, float cy, float ballRadius, const vector<ProPoint>& contour, int segmentIndex, float scaleX, float scaleY, const ProRect& objectBounds,
   bool centerInside, Contact& contact)
{
   const size_t next = (static_cast<size_t>(segmentIndex) + 1) % contour.size();
   const float sx = objectBounds.x + contour[segmentIndex].x * scaleX, sy = objectBounds.y + contour[segmentIndex].y * scaleY;
   const float ex = objectBounds.x + contour[next].x * scaleX, ey = objectBounds.y + contour[next].y * scaleY;
   const float segX = ex - sx, segY = ey - sy;
   const float lengthSquared = segX * segX + segY * segY;
   if (lengthSquared <= 0.000001f)
      return false;
   const float fraction = std::clamp(((cx - sx) * segX + (cy - sy) * segY) / lengthSquared, 0.f, 1.f);
   const float nearestX = sx + segX * fraction, nearestY = sy + segY * fraction;
   const float dx = cx - nearestX, dy = cy - nearestY;
   const float distance = static_cast<float>(std::sqrt(dx * dx + dy * dy));
   if (!centerInside && distance > ballRadius)
      return false;
   float nx, ny;
   if (distance > 0.0001f)
   {
      if (centerInside)
      {
         nx = (nearestX - cx) / distance;
         ny = (nearestY - cy) / distance;
      }
      else
      {
         nx = dx / distance;
         ny = dy / distance;
      }
   }
   else
   {
      const float length = static_cast<float>(std::sqrt(lengthSquared));
      nx = segY / length;
      ny = -segX / length;
   }
   contact.contactX = nearestX;
   contact.contactY = nearestY;
   contact.normalX = nx;
   contact.normalY = ny;
   contact.penetration = centerInside ? ballRadius + distance : ballRadius - distance;
   contact.distance = distance;
   contact.segmentIndex = segmentIndex;
   return true;
}

static bool ClipPivotSweepAxis(float start, float movement, float minimum, float maximum, float minNormalX, float minNormalY, float& enterTime, float& exitTime, float& nx, float& ny)
{
   if (std::abs(movement) < 0.000001f)
      return start >= minimum && start <= maximum;
   float first = (minimum - start) / movement, last = (maximum - start) / movement;
   float fnx = minNormalX, fny = minNormalY;
   if (first > last)
   {
      std::swap(first, last);
      fnx = -minNormalX;
      fny = -minNormalY;
   }
   if (first > enterTime)
   {
      enterTime = first;
      nx = fnx;
      ny = fny;
   }
   if (last < exitTime)
      exitTime = last;
   return enterTime <= exitTime && exitTime >= 0.f && enterTime <= 1.f;
}

void B2SProPhysics::ResolveKnockdownPivot(ProPoint& center, ProPoint previous, float ballRadius, ProPoint& velocity, B2SBulb& pivot, const ProRect& objectBounds, float bounce)
{
   const B2SProBulbData& pp = *pivot.m_pro;
   const ProBulbRuntime& prt = pivot.m_pro->rt;
   const ProPoint hinge { objectBounds.x + objectBounds.w * pp.pivotX, objectBounds.y + objectBounds.h * pp.pivotY };
   const double radians = prt.angle * kPi / 180.0;
   const float cosine = static_cast<float>(std::cos(radians)), sine = static_cast<float>(std::sin(radians));
   const float pox = previous.x - hinge.x, poy = previous.y - hinge.y, cox = center.x - hinge.x, coy = center.y - hinge.y;
   const float plx = hinge.x + pox * cosine + poy * sine, ply = hinge.y - pox * sine + poy * cosine;
   const float clx = hinge.x + cox * cosine + coy * sine, cly = hinge.y - cox * sine + coy * cosine;
   const float eLeft = objectBounds.x - ballRadius, eTop = objectBounds.y - ballRadius;
   const float eRight = objectBounds.x + objectBounds.w + ballRadius, eBottom = objectBounds.y + objectBounds.h + ballRadius;
   const float mx = clx - plx, my = cly - ply;
   float hitTime = 0.f, exitTime = 1.f, lnx = 0.f, lny = 0.f;
   if (!ClipPivotSweepAxis(plx, mx, eLeft, eRight, -1.f, 0.f, hitTime, exitTime, lnx, lny))
      return;
   if (!ClipPivotSweepAxis(ply, my, eTop, eBottom, 0.f, -1.f, hitTime, exitTime, lnx, lny))
      return;
   if (hitTime < 0.f || hitTime > 1.f)
      return;
   float ilx = plx + mx * hitTime, ily = ply + my * hitTime;
   if (std::abs(lnx) < 0.001f && std::abs(lny) < 0.001f)
   {
      const float l = std::abs(ilx - eLeft), r = std::abs(eRight - ilx), t = std::abs(ily - eTop), b = std::abs(eBottom - ily);
      const float nearest = std::min(std::min(l, r), std::min(t, b));
      if (nearest == l)
         lnx = -1.f;
      else if (nearest == r)
         lnx = 1.f;
      else if (nearest == t)
         lny = -1.f;
      else
         lny = 1.f;
      // Already overlapping: move to the nearest expanded edge
      if (lnx < -0.5f)
         ilx = eLeft;
      else if (lnx > 0.5f)
         ilx = eRight;
      else if (lny < -0.5f)
         ily = eTop;
      else if (lny > 0.5f)
         ily = eBottom;
   }
   const float iox = ilx - hinge.x, ioy = ily - hinge.y;
   center.x = hinge.x + iox * cosine - ioy * sine;
   center.y = hinge.y + iox * sine + ioy * cosine;
   const float nx = lnx * cosine - lny * sine, ny = lnx * sine + lny * cosine;
   center.x += nx * 0.01f;
   center.y += ny * 0.01f;
   const float contactX = center.x - nx * ballRadius, contactY = center.y - ny * ballRadius;
   const float armX = contactX - hinge.x, armY = contactY - hinge.y;
   const float angularRadians = prt.physicsAngularVelocity * static_cast<float>(kPi / 180.0);
   const float sx = -angularRadians * armY, sy = angularRadians * armX;
   const float incomingNormalSpeed = (velocity.x - sx) * nx + (velocity.y - sy) * ny;
   if (incomingNormalSpeed >= 0.f)
      return;
   ReflectBoundaryVelocity(velocity, nx, ny, bounce, incomingNormalSpeed);
   ApplyPivotPhysicsAngularImpulse(pivot, 1.f);
}

void B2SProPhysics::ResolveStaticFloor(ProPoint& center, ProPoint previous, float radius, ProPoint& velocity, ProPoint from, ProPoint to, float bounce)
{
   const float segX = to.x - from.x, segY = to.y - from.y;
   const float lengthSquared = segX * segX + segY * segY;
   if (lengthSquared <= 0.01f)
      return;
   const float rawFraction = ((center.x - from.x) * segX + (center.y - from.y) * segY) / lengthSquared;
   const float fraction = std::clamp(rawFraction, 0.f, 1.f);
   const float contactX = from.x + segX * fraction, contactY = from.y + segY * fraction;
   const float ox = center.x - contactX, oy = center.y - contactY;
   const float distanceSquared = ox * ox + oy * oy;
   if (distanceSquared >= radius * radius)
   {
      // A fast ball can cross the boundary between steps: sweep against the segment capsule
      ProPoint sweptCenter, sweptNormal;
      if (!TrySweptStaticFloor(previous, center, radius, from, to, sweptCenter, sweptNormal))
         return;
      center.x = sweptCenter.x + sweptNormal.x * 0.01f;
      center.y = sweptCenter.y + sweptNormal.y * 0.01f;
      const float sweptNormalSpeed = velocity.x * sweptNormal.x + velocity.y * sweptNormal.y;
      if (sweptNormalSpeed < 0.f)
         ReflectBoundaryVelocity(velocity, sweptNormal.x, sweptNormal.y, bounce, sweptNormalSpeed);
      return;
   }
   const float distance = static_cast<float>(std::sqrt(distanceSquared));
   float nx, ny;
   if (rawFraction < 0.f || rawFraction > 1.f)
   {
      const float pox = previous.x - contactX, poy = previous.y - contactY;
      const float previousDistance = static_cast<float>(std::sqrt(pox * pox + poy * poy));
      if (distance >= 0.001f)
      {
         nx = ox / distance;
         ny = oy / distance;
      }
      else if (previousDistance >= 0.001f)
      {
         nx = pox / previousDistance;
         ny = poy / previousDistance;
      }
      else
         return;
   }
   else
   {
      const float segmentLength = static_cast<float>(std::sqrt(lengthSquared));
      nx = segY / segmentLength;
      ny = -segX / segmentLength;
      float previousSide = (previous.x - contactX) * nx + (previous.y - contactY) * ny;
      if (std::abs(previousSide) < 0.001f)
         previousSide = ox * nx + oy * ny;
      if (std::abs(previousSide) < 0.001f)
      {
         if (velocity.x * nx + velocity.y * ny > 0.f)
         {
            nx = -nx;
            ny = -ny;
         }
      }
      else if (previousSide < 0.f)
      {
         nx = -nx;
         ny = -ny;
      }
   }
   center.x = contactX + nx * radius;
   center.y = contactY + ny * radius;
   const float normalSpeed = velocity.x * nx + velocity.y * ny;
   if (normalSpeed < 0.f)
      ReflectBoundaryVelocity(velocity, nx, ny, bounce, normalSpeed);
}

void B2SProPhysics::ReflectBoundaryVelocity(ProPoint& velocity, float nx, float ny, float bounce, float incomingNormalSpeed)
{
   velocity.x -= (1.f + bounce) * incomingNormalSpeed * nx;
   velocity.y -= (1.f + bounce) * incomingNormalSpeed * ny;
   if (-incomingNormalSpeed < kBoundaryScatterMinimumImpactSpeed)
      return;
   std::uniform_real_distribution<double> unit(0.0, 1.0);
   const float scatterDegrees = static_cast<float>((unit(m_scatterRandom) * 2.0 - 1.0) * kBoundaryScatterDegrees);
   // RotateBoundaryVelocity: keep the scatter only when it still leaves the surface
   const double radians = scatterDegrees * kPi / 180.0;
   const float c = static_cast<float>(std::cos(radians)), s = static_cast<float>(std::sin(radians));
   const float rx = velocity.x * c - velocity.y * s, ry = velocity.x * s + velocity.y * c;
   if (rx * nx + ry * ny <= 0.f)
      return;
   velocity = { rx, ry };
}

bool B2SProPhysics::TrySweptStaticFloor(ProPoint previous, ProPoint current, float radius, ProPoint from, ProPoint to, ProPoint& impactCenter, ProPoint& impactNormal) const
{
   const float mx = current.x - previous.x, my = current.y - previous.y;
   const float motionLengthSquared = mx * mx + my * my;
   if (motionLengthSquared <= 0.000001f)
      return false;
   const float segX = to.x - from.x, segY = to.y - from.y;
   const float segmentLengthSquared = segX * segX + segY * segY;
   if (segmentLengthSquared <= 0.01f)
      return false;
   const float segmentLength = static_cast<float>(std::sqrt(segmentLengthSquared));
   const float tx = segX / segmentLength, ty = segY / segmentLength;
   const float bnx = ty, bny = -tx;
   const float previousSigned = (previous.x - from.x) * bnx + (previous.y - from.y) * bny;
   const float currentSigned = (current.x - from.x) * bnx + (current.y - from.y) * bny;
   const float signedChange = currentSigned - previousSigned;
   float bestTime = std::numeric_limits<float>::max();
   bool found = false;
   // Embedded by a neighbouring segment and now crossing: restore the side it was on
   const float previousProjection = (previous.x - from.x) * tx + (previous.y - from.y) * ty;
   const float currentProjection = (current.x - from.x) * tx + (current.y - from.y) * ty;
   if (previousProjection >= 0.f && previousProjection <= segmentLength && currentProjection >= 0.f && currentProjection <= segmentLength && std::abs(previousSigned) < radius
      && previousSigned * currentSigned <= 0.f && std::abs(currentSigned) >= radius)
   {
      float retainedSide;
      if (std::abs(previousSigned) >= 0.000001f)
         retainedSide = previousSigned > 0.f ? 1.f : -1.f;
      else
         retainedSide = signedChange < 0.f ? 1.f : -1.f;
      impactCenter = { from.x + tx * previousProjection + bnx * retainedSide * radius, from.y + ty * previousProjection + bny * retainedSide * radius };
      impactNormal = { bnx * retainedSide, bny * retainedSide };
      return true;
   }
   for (int side = 0; side <= 1; side++)
   {
      const float sign = side == 0 ? 1.f : -1.f;
      if (sign * signedChange < -0.000001f)
      {
         const float hitTime = (sign * radius - previousSigned) / signedChange;
         if (hitTime >= 0.f && hitTime <= 1.f && hitTime < bestTime)
         {
            const float hx = previous.x + mx * hitTime, hy = previous.y + my * hitTime;
            const float projection = (hx - from.x) * tx + (hy - from.y) * ty;
            if (projection >= 0.f && projection <= segmentLength)
            {
               bestTime = hitTime;
               impactCenter = { hx, hy };
               impactNormal = { bnx * sign, bny * sign };
               found = true;
            }
         }
      }
   }
   for (int endpointIndex = 0; endpointIndex <= 1; endpointIndex++)
   {
      const ProPoint endpoint = endpointIndex == 0 ? from : to;
      const float rx = previous.x - endpoint.x, ry = previous.y - endpoint.y;
      const float circleConstant = rx * rx + ry * ry - radius * radius;
      if (circleConstant < 0.f)
         continue;
      const float circleLinear = 2.f * (rx * mx + ry * my);
      const double discriminant = static_cast<double>(circleLinear) * circleLinear - 4.0 * static_cast<double>(motionLengthSquared) * circleConstant;
      if (discriminant < 0.0)
         continue;
      const float hitTime = static_cast<float>((-static_cast<double>(circleLinear) - std::sqrt(discriminant)) / (2.0 * motionLengthSquared));
      if (hitTime >= 0.f && hitTime <= 1.f && hitTime < bestTime)
      {
         const float hx = previous.x + mx * hitTime, hy = previous.y + my * hitTime;
         const float nx = hx - endpoint.x, ny = hy - endpoint.y;
         const float length = static_cast<float>(std::sqrt(nx * nx + ny * ny));
         if (length >= 0.001f)
         {
            bestTime = hitTime;
            impactCenter = { hx, hy };
            impactNormal = { nx / length, ny / length };
            found = true;
         }
      }
   }
   return found;
}

void B2SProPhysics::ResolveFlipperCollision(Ball& ball, ProPoint& center, float ballRadius, ProPoint& velocity, B2SBulb& flipper, float angularVelocity)
{
   const B2SProBulbData& fp = *flipper.m_pro;
   const ProRect fb = PivotBounds(flipper);
   const ProPoint hinge { fb.x + fb.w * fp.pivotX, fb.y + fb.h * fp.pivotY };
   const float baseTipX = fb.x + fb.w * fp.pivotTipX, baseTipY = fb.y + fb.h * fp.pivotTipY;
   const double radians = flipper.m_pro->rt.angle * kPi / 180.0;
   const double sx = baseTipX - hinge.x, sy = baseTipY - hinge.y;
   const ProPoint tip { static_cast<float>(hinge.x + sx * std::cos(radians) - sy * std::sin(radians)), static_cast<float>(hinge.y + sx * std::sin(radians) + sy * std::cos(radians)) };
   const float segX = tip.x - hinge.x, segY = tip.y - hinge.y;
   const float lengthSquared = segX * segX + segY * segY;
   if (lengthSquared <= 0.01f)
      return;
   const float rawFraction = ((center.x - hinge.x) * segX + (center.y - hinge.y) * segY) / lengthSquared;
   const float fraction = std::clamp(rawFraction, 0.f, 1.f);
   const ProPoint contact { hinge.x + segX * fraction, hinge.y + segY * fraction };
   const float segmentLength = static_cast<float>(std::sqrt(lengthSquared));
   float nx = segY / segmentLength, ny = -segX / segmentLength;
   if (ny > 0.f)
   {
      nx = -nx;
      ny = -ny;
   }
   const float flipperRadius = std::max(3.f, std::min(fb.w, fb.h) * 0.07f);
   const float minimumDistance = ballRadius + flipperRadius;
   if (rawFraction < 0.f || rawFraction > 1.f)
   {
      const float ex = center.x - contact.x, ey = center.y - contact.y;
      const float endDistance = static_cast<float>(std::sqrt(ex * ex + ey * ey));
      if (endDistance >= minimumDistance || endDistance < 0.001f)
         return;
      nx = ex / endDistance;
      ny = ey / endDistance;
      if (ny > 0.f)
         return;
      center.x += nx * (minimumDistance - endDistance);
      center.y += ny * (minimumDistance - endDistance);
   }
   else
   {
      const float signedDistance = (center.x - contact.x) * nx + (center.y - contact.y) * ny;
      if (signedDistance >= minimumDistance)
         return;
      center.x += nx * (minimumDistance - signedDistance);
      center.y += ny * (minimumDistance - signedDistance);
   }
   const float armX = contact.x - hinge.x, armY = contact.y - hinge.y;
   float impactStrength = ball.flipperStrength;
   float variation = fp.pivotRandomStrength;
   if (angularVelocity != 0.f && variation > 0.f && !std::isinf(variation))
   {
      variation = std::min(100.f, variation);
      std::uniform_real_distribution<double> unit(0.0, 1.0);
      impactStrength *= 1.f + static_cast<float>((unit(m_strengthRandom) * 2.0 - 1.0) * variation / 100.0);
   }
   const float surfaceX = -angularVelocity * armY * impactStrength, surfaceY = angularVelocity * armX * impactStrength;
   float relativeX = velocity.x - surfaceX, relativeY = velocity.y - surfaceY;
   const float towardSurface = relativeX * nx + relativeY * ny;
   if (towardSurface < 0.f)
   {
      constexpr float restitution = 0.58f;
      relativeX -= (1.f + restitution) * towardSurface * nx;
      relativeY -= (1.f + restitution) * towardSurface * ny;
      velocity = { surfaceX + relativeX, surfaceY + relativeY };
   }
}

// ------------------------------------------------------------------------------------------------
// B2SPictureBox physics pivot members

void B2SProPhysics::AdvancePhysicsPivot(B2SBulb& pivot, double elapsed)
{
   const B2SProBulbData& pp = *pivot.m_pro;
   ProBulbRuntime& rt = pivot.m_pro->rt;
   rt.physicsPreviousAngle = rt.angle;
   if (pp.pivotPhysicsEnabled && pp.pivotPhysicsResponseMode > 0)
   {
      // AdvancePivotPhysicsResponse
      if (elapsed > 0.0 && rt.responseState != 0 && rt.responseState != 2)
      {
         rt.responseElapsedMs += elapsed * 1000.0;
         if (rt.responseState == 1)
         {
            const double duration = std::clamp(pp.pivotPhysicsKnockdownDuration, 10, 5000);
            const float progress = static_cast<float>(std::min(1.0, rt.responseElapsedMs / duration));
            const float eased = progress * progress * (3.f - 2.f * progress);
            rt.angle = rt.responseStartAngle + (pp.pivotPhysicsKnockdownAngle - rt.responseStartAngle) * eased;
            if (progress >= 1.f)
            {
               rt.angle = pp.pivotPhysicsKnockdownAngle;
               rt.responseElapsedMs = 0.0;
               rt.responseState = pp.pivotPhysicsResponseMode == 2 ? 3 : 2;
            }
         }
         else
         {
            bool proceed = true;
            if (rt.responseState == 3)
            {
               if (rt.responseElapsedMs < std::clamp(pp.pivotPhysicsResetDelay, 0, 30000))
                  proceed = false;
               else
               {
                  rt.responseStartAngle = rt.angle;
                  rt.responseElapsedMs = 0.0;
                  rt.responseState = 4;
               }
            }
            if (proceed && rt.responseState == 4)
            {
               const double duration = std::clamp(pp.pivotPhysicsKnockdownDuration, 10, 5000);
               const float progress = static_cast<float>(std::min(1.0, rt.responseElapsedMs / duration));
               const float eased = progress * progress * (3.f - 2.f * progress);
               rt.angle = rt.responseStartAngle + (rt.responseRestAngle - rt.responseStartAngle) * eased;
               if (progress >= 1.f)
               {
                  rt.angle = rt.responseRestAngle;
                  rt.responseElapsedMs = 0.0;
                  rt.responseState = 0;
               }
            }
         }
      }
      rt.physicsAngularVelocity = elapsed > 0.0 ? static_cast<float>((rt.angle - rt.physicsPreviousAngle) / elapsed) : 0.f;
      return;
   }
   if (rt.physicsMoving)
   {
      rt.physicsElapsedMs += elapsed * 1000.0;
      const float progress = static_cast<float>(std::min(1.0, rt.physicsElapsedMs / rt.tweenDuration));
      const float eased = progress * progress * (3.f - 2.f * progress);
      rt.angle = rt.startAngle + (rt.targetAngle - rt.startAngle) * eased;
      if (progress >= 1.f)
      {
         rt.angle = rt.targetAngle;
         rt.physicsMoving = false;
         if (rt.automaticOscillation && rt.automaticActive)
            SetPivotRotationTarget(rt, std::abs(rt.targetAngle - rt.upAngle) < 0.01f ? rt.downAngle : rt.upAngle, rt.moveDuration);
      }
      return;
   }
   if (!pp.pivotPhysicsEnabled || elapsed <= 0.0 || std::abs(rt.physicsAngularVelocity) < 0.0001f)
      return;
   rt.angle = static_cast<float>(std::fmod(rt.angle + rt.physicsAngularVelocity * elapsed, 360.0));
   const double damping = std::exp(-std::max(0.f, pp.pivotPhysicsFriction) * elapsed);
   rt.physicsAngularVelocity = static_cast<float>(rt.physicsAngularVelocity * damping);
   if (std::abs(rt.physicsAngularVelocity) < 0.01f)
      rt.physicsAngularVelocity = 0.f;
}

void B2SProPhysics::ApplyPivotPhysicsAngularImpulse(B2SBulb& pivot, float deltaDegreesPerSecond)
{
   const B2SProBulbData& pp = *pivot.m_pro;
   ProBulbRuntime& rt = pivot.m_pro->rt;
   if (!pp.pivotPhysicsEnabled || std::isnan(deltaDegreesPerSecond) || std::isinf(deltaDegreesPerSecond))
      return;
   rt.moving = false;
   rt.physicsMoving = false;
   if (pp.pivotPhysicsResponseMode > 0)
   {
      if (rt.responseState != 0)
         return;
      rt.physicsAngularVelocity = 0.f;
      rt.responseRestAngle = rt.angle;
      rt.responseStartAngle = rt.angle;
      rt.responseElapsedMs = 0.0;
      rt.responseState = 1;
      rt.pivotRotation = true;
      rt.nativeRotation = true;
      rt.driven = true;
      rt.visible = true;
      return;
   }
   rt.physicsAngularVelocity = std::clamp(rt.physicsAngularVelocity + deltaDegreesPerSecond, -2160.f, 2160.f);
   rt.pivotRotation = true;
   rt.nativeRotation = true;
   rt.driven = true;
   rt.visible = true;
}

// SetPivotRotationTarget of a physics pivot: the tween advances in the physics steps
void B2SProPhysics::SetPivotRotationTarget(ProBulbRuntime& rt, float targetAngle, int duration)
{
   rt.physicsAngularVelocity = 0.f;
   rt.startAngle = rt.angle;
   rt.targetAngle = targetAngle;
   rt.tweenDuration = std::clamp(duration, 10, rt.automaticOscillation ? 5000 : 1000);
   rt.pivotRotation = true;
   rt.nativeRotation = true;
   rt.driven = true;
   rt.visible = true;
   rt.physicsElapsedMs = 0.0;
   rt.moving = false;
   rt.physicsMoving = true;
}

}
