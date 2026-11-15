// license:GPLv3+

#include "B2SProMotion.h"
#include "B2SServer.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <random>

namespace B2S {

static constexpr double kPi = 3.14159265358979323846;

static ProMotion& M(B2SBulb& bulb) { return bulb.m_pro->rt.motion; }
static const ProMotion& M(const B2SBulb& bulb) { return bulb.m_pro->rt.motion; }
static ProRect Rect(const B2SBulb& bulb)
{
   const ProMotion& m = M(bulb);
   return { m.x, m.y, m.w, m.h };
}
static ProPoint RectCenter(const ProRect& r) { return { r.x + r.w / 2.f, r.y + r.h / 2.f }; }
static bool Contains(const vector<B2SBulb*>& v, const B2SBulb* b) { return std::find(v.begin(), v.end(), b) != v.end(); }
static void Remove(vector<B2SBulb*>& v, const B2SBulb* b)
{
   const auto it = std::find(v.begin(), v.end(), b);
   if (it != v.end())
      v.erase(it);
}
static void AddUnique(vector<B2SBulb*>& v, B2SBulb* b)
{
   if (!Contains(v, b))
      v.push_back(b);
}
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
static double PathLength(const vector<ProPoint>& points)
{
   double total = 0.0;
   for (size_t i = 0; i + 1 < points.size(); i++)
   {
      const double dx = points[i + 1].x - points[i].x, dy = points[i + 1].y - points[i].y;
      total += std::sqrt(dx * dx + dy * dy);
   }
   return total;
}
static float FirstSegmentCompletion(const vector<ProPoint>& points)
{
   if (points.size() < 2)
      return 1.f;
   double total = 0.0, firstDistinctEnd = -1.0;
   for (size_t i = 0; i + 1 < points.size(); i++)
   {
      const double dx = points[i + 1].x - points[i].x, dy = points[i + 1].y - points[i].y;
      const double length = std::sqrt(dx * dx + dy * dy);
      total += length;
      if (firstDistinctEnd < 0.0 && length > 0.001)
         firstDistinctEnd = total;
   }
   if (total <= 0.001 || firstDistinctEnd < 0.0)
      return 1.f;
   return static_cast<float>(std::clamp(firstDistinctEnd / total, 0.0, 1.0));
}
static void AddDistinct(vector<ProPoint>& route, ProPoint p)
{
   if (route.empty())
   {
      route.push_back(p);
      return;
   }
   const double dx = p.x - route.back().x, dy = p.y - route.back().y;
   if (dx * dx + dy * dy > 0.0001)
      route.push_back(p);
}

ProPoint B2SProMotionSystem::Center(const B2SBulb& bulb) { return RectCenter(Rect(bulb)); }

void B2SProMotionSystem::InitRect(B2SBulb& bulb)
{
   ProMotion& m = M(bulb);
   const B2SProBulbData& pro = *bulb.m_pro;
   m.x = static_cast<float>(bulb.m_locationX);
   m.y = static_cast<float>(bulb.m_locationY);
   m.w = static_cast<float>(bulb.m_width);
   m.h = static_cast<float>(bulb.m_height);
   m.startW = m.w;
   m.startH = m.h;
   m.image = bulb.m_image;
   if (pro.motionPathPoints.size() >= 2)
      m.authoredPoints = pro.motionPathPoints;
   m.sequenceOrder = pro.motionPathSequenceOrder;
}

B2SProMotionSystem::B2SProMotionSystem(B2STable& table)
   : m_table(table)
{
   for (auto* bulbs : { &table.m_backglassIlluminations, &table.m_dmdIlluminations })
      for (auto& bulbPtr : *bulbs)
         if (bulbPtr->m_pro)
            InitRect(*bulbPtr);

   for (auto* bulbs : { &table.m_backglassIlluminations, &table.m_dmdIlluminations })
      for (auto& bulbPtr : *bulbs)
      {
         B2SBulb* const bulb = bulbPtr.get();
         if (!bulb->m_pro)
            continue;
         const B2SProBulbData& pro = *bulb->m_pro;
         ProMotion& m = M(*bulb);
         const bool hasPath = m.authoredPoints.size() >= 2;
         const bool hasExit = pro.motionPathExitPoints.size() >= 2;

         // B2SData.RegisterMotionPathSequence
         if (!pro.motionPathSequenceGroup.empty() && pro.motionPathSequenceOrder > 0)
         {
            const string key = LowerTrim(pro.motionPathSequenceGroup);
            size_t index;
            if (const auto it = m_sequenceByName.find(key); it != m_sequenceByName.end())
               index = it->second;
            else
            {
               index = m_sequences.size();
               m_sequences.emplace_back();
               m_sequenceByName[key] = index;
            }
            m.sequence = static_cast<int>(index);
            SequenceRegister(m_sequences[index], bulb);
         }

         // B2SData.RegisterExternalPositionGrid
         if (!pro.externalGridName.empty() && !LowerTrim(pro.name).empty())
         {
            const string gridKey = LowerTrim(pro.externalGridName);
            size_t index;
            if (const auto it = m_gridByName.find(gridKey); it != m_gridByName.end())
               index = it->second;
            else
            {
               index = m_grids.size();
               m_grids.emplace_back();
               m_gridByName[gridKey] = index;
            }
            Grid& grid = m_grids[index];
            const string targetKey = LowerTrim(pro.name);
            grid.targets[targetKey] = bulb;
            if (bulb != grid.representative)
               bulb->m_pro->rt.visible = false;
            if (pro.externalGridRepresentative)
               grid.representative = bulb;
            grid.duration = std::clamp(pro.externalGridDuration, 10, 1000);
            m_gridTargets[targetKey] = index;
         }

         // Trigger routes (formBackglass.LoadB2SData / RegisterMotionControlRoutes / RegisterSwitchTriggerRoutes)
         const auto add = [this](int type, int id, vector<B2SBulb*> Route::*list, B2SBulb* b)
         {
            if (id <= 0)
               return;
            (m_routes[{ type, id }].*list).push_back(b);
            if (type != 3)
               m_edges[{ type, id }];
         };
         if (hasPath)
         {
            add(1, pro.motionPathSolenoidId, &Route::start, bulb);
            add(2, pro.motionPathLampId, &Route::start, bulb);
            add(3, pro.motionPathB2SId, &Route::start, bulb);
            add(5, pro.motionPathSwitchId, &Route::start, bulb);
            add(3, pro.motionPathStopB2SId, &Route::stop, bulb);
            add(3, pro.motionPathResumeB2SId, &Route::resume, bulb);
            add(2, pro.motionPathStopLampId, &Route::stop, bulb);
            add(2, pro.motionPathResumeLampId, &Route::resume, bulb);
            add(1, pro.motionPathStopSolenoidId, &Route::stop, bulb);
            add(1, pro.motionPathResumeSolenoidId, &Route::resume, bulb);
            add(5, pro.motionPathStopSwitchId, &Route::stop, bulb);
            add(5, pro.motionPathResumeSwitchId, &Route::resume, bulb);
            if (pro.motionPathSolenoidId == 0 && pro.motionPathLampId == 0 && pro.motionPathB2SId == 0 && pro.motionPathSwitchId == 0)
               m_startupPending[bulb] = true;
         }
         if (hasExit || pro.motionPathGravityDrop)
         {
            add(1, pro.motionPathRemoveSolenoidId, &Route::remove, bulb);
            add(2, pro.motionPathRemoveLampId, &Route::remove, bulb);
            add(3, pro.motionPathRemoveB2SId, &Route::remove, bulb);
            add(5, pro.motionPathRemoveSwitchId, &Route::remove, bulb);
         }

         m.active = hasPath || hasExit || m.sequence >= 0 || pro.physicsBall || (!pro.externalGridName.empty() && pro.externalGridRepresentative);
      }
}

void B2SProMotionSystem::BindTriggers(const Resolver& resolver)
{
   for (auto& [key, edge] : m_edges)
   {
      const B2SRomIDType type = key.first == 1 ? B2SRomIDType::Solenoid : key.first == 2 ? B2SRomIDType::Lamp : B2SRomIDType::Switch;
      edge.updater = resolver(type, key.second, &edge.value);
   }
}

void B2SProMotionSystem::FireRoute(int type, int id, bool scriptOrder)
{
   const auto it = m_routes.find({ type, id });
   if (it == m_routes.end())
      return;
   const Route route = it->second; // Handlers may change motion state, not the routes
   const auto doStart = [&]() { if (!route.start.empty()) StartMotionPaths(route.start); };
   const auto doRemove = [&]() { if (!route.remove.empty()) RemoveMotionPaths(route.remove); };
   const auto doStop = [&]() { for (B2SBulb* b : route.stop) StopMotionPath(*b); };
   const auto doResume = [&]() { for (B2SBulb* b : route.resume) ResumeMotionPath(*b); };
   if (scriptOrder)
   {
      // MyB2SSetData / ApplySwitchTriggers: start, remove, stop, resume
      doStart();
      doRemove();
      doStop();
      doResume();
   }
   else
   {
      // ChangedLamps / ChangedSolenoids: stop, resume, start, remove
      doStop();
      doResume();
      doStart();
      doRemove();
   }
}

void B2SProMotionSystem::PollTriggers(B2SServer* server)
{
   for (auto& [key, edge] : m_edges)
   {
      edge.updater();
      if (edge.value < 0.f)
         continue;
      const bool isSwitch = key.first == 5;
      const bool on = isSwitch ? edge.value > 0.f : edge.value >= 0.5f;
      const bool known = edge.last >= 0.f;
      const bool wasOn = known && (isSwitch ? edge.last > 0.f : edge.last >= 0.5f);
      edge.last = edge.value;
      if (on && (!known || !wasOn))
         FireRoute(key.first, key.second, isSwitch);
   }
   if (server == nullptr)
      return;
   for (const auto& [key, route] : m_routes)
   {
      if (key.first != 3)
         continue;
      float value = 0.f;
      uint64_t stamp = 0;
      if (!server->GetScriptedLampState(key.second, value, &stamp))
         continue;
      auto& state = m_b2sStates[key.second];
      if (stamp == state.first)
         continue;
      state.first = stamp;
      const int previous = state.second;
      state.second = static_cast<int>(value);
      if (previous == 0 && state.second != 0)
         FireRoute(3, key.second, true);
   }
}

bool B2SProMotionSystem::TryMoveGrid(const string& key, int value)
{
   const auto it = m_gridTargets.find(key);
   if (it == m_gridTargets.end())
      return false;
   if (value == 0)
      return true;
   Grid& grid = m_grids[it->second];
   if (grid.representative == nullptr || grid.targets.find(key) == grid.targets.end())
      return true;
   // ExternalPositionGridState.MoveTo: a short look-ahead window keeps consecutive markers one curve
   grid.flushOn = false;
   const double interval = std::clamp(grid.duration * 2, 20, 2000);
   if (!grid.pending.empty() && grid.targets.find(grid.pending) != grid.targets.end())
      StartExternalGridShift(*grid.representative, Center(*grid.targets[grid.pending]), grid.duration, Center(*grid.targets[key]));
   grid.pending = key;
   grid.flushRemainingMs = interval;
   grid.flushOn = true;
   return true;
}

void B2SProMotionSystem::Update(const vector<std::unique_ptr<B2SBulb>>& bulbs, float elapsedS)
{
   const double elapsedMs = static_cast<double>(elapsedS) * 1000.0;
   for (const auto& bulbPtr : bulbs)
   {
      if (!bulbPtr->m_pro)
         continue;
      B2SBulb& bulb = *bulbPtr;
      if (const auto it = m_startupPending.find(&bulb); it != m_startupPending.end())
      {
         m_startupPending.erase(it);
         StartMotionPath(bulb);
      }
      Tick(bulb, elapsedMs);
   }
   // Grid flush timers (FlushPendingTarget) of this window's grids
   for (Grid& grid : m_grids)
   {
      if (!grid.flushOn || grid.representative == nullptr)
         continue;
      if (std::find_if(bulbs.begin(), bulbs.end(), [&grid](const std::unique_ptr<B2SBulb>& b) { return b.get() == grid.representative; }) == bulbs.end())
         continue;
      grid.flushRemainingMs -= elapsedMs;
      if (grid.flushRemainingMs > 0.0)
         continue;
      grid.flushOn = false;
      if (grid.pending.empty() || grid.targets.find(grid.pending) == grid.targets.end())
         continue;
      const ProPoint target = Center(*grid.targets[grid.pending]);
      grid.pending.clear();
      StartExternalGridShift(*grid.representative, target, grid.duration, target);
   }
}

// ------------------------------------------------------------------------------------------------
// B2SPictureBox motion members

void B2SProMotionSystem::SetCenter(B2SBulb& bulb, ProPoint center, bool advanceRoll)
{
   ProMotion& m = M(bulb);
   const ProPoint previous = Center(bulb);
   if (advanceRoll)
      AdvanceRoll(bulb, previous, center, m.w, m.h, m.rollAngle);
   m.x = center.x - m.startW / 2.f;
   m.y = center.y - m.startH / 2.f;
   m.w = m.startW;
   m.h = m.startH;
}

void B2SProMotionSystem::AdvanceRoll(const B2SBulb& bulb, ProPoint previous, ProPoint current, float visualW, float visualH, float& angle) const
{
   if (!bulb.m_pro->motionPathRollEnabled)
      return;
   const float dx = current.x - previous.x, dy = current.y - previous.y;
   const double distance = std::sqrt(static_cast<double>(dx) * dx + static_cast<double>(dy) * dy);
   if (distance <= 0.0001)
      return;
   const float direction = std::abs(dx) >= std::abs(dy) ? static_cast<float>((dx > 0.f) - (dx < 0.f)) : static_cast<float>((dy > 0.f) - (dy < 0.f));
   if (direction == 0.f)
      return;
   const float radius = std::max(1.f, std::min(visualW, visualH) / 2.f);
   angle = static_cast<float>(std::fmod(angle + direction * distance / radius * 180.0 / kPi, 360.0));
}

void B2SProMotionSystem::StopCollision(B2SBulb& bulb, bool restorePosition)
{
   ProMotion& m = M(bulb);
   m.collisionOn = false;
   m.collisionClockMs = 0.0;
   if (restorePosition && !m.collisionBase.IsEmpty())
   {
      m.x = m.collisionBase.x;
      m.y = m.collisionBase.y;
      m.w = m.collisionBase.w;
      m.h = m.collisionBase.h;
   }
   m.collisionBase = {};
   m.collisionDir = {};
   m.collisionAmp = 0.f;
   m.collisionDelay = 0;
}

void B2SProMotionSystem::StartCollision(B2SBulb& bulb, ProPoint direction, float amplitude, int delayMs)
{
   StopCollision(bulb, true);
   ProMotion& m = M(bulb);
   const double length = std::sqrt(static_cast<double>(direction.x) * direction.x + static_cast<double>(direction.y) * direction.y);
   if (length <= 0.001 || amplitude <= 0.f)
      return;
   m.collisionBase = Rect(bulb);
   m.collisionDir = { static_cast<float>(direction.x / length), static_cast<float>(direction.y / length) };
   m.collisionAmp = amplitude;
   m.collisionDelay = std::max(0, delayMs);
   m.collisionClockMs = 0.0;
   m.collisionOn = true;
}

void B2SProMotionSystem::StartCore(B2SBulb& bulb, bool useExitPath, bool startFromCurrentPosition)
{
   const B2SProBulbData& pro = *bulb.m_pro;
   ProMotion& m = M(bulb);
   const vector<ProPoint>& sourcePoints = useExitPath ? pro.motionPathExitPoints : m.authoredPoints;
   if (sourcePoints.size() < 2)
      return;
   if (!useExitPath && pro.motionPathQueueTriggers && (m.timerOn || m.paused))
   {
      m.queuedStarts = std::min(32, m.queuedStarts + 1);
      return;
   }
   StopCollision(bulb, true);
   m.timerOn = false;
   m.clockMs = 0.0;
   m.paused = false;
   m.externalShift = false;
   m.activeDuration = std::clamp(useExitPath ? pro.motionPathExitDuration : pro.motionPathDuration, 250, 30000);
   m.startDelay = 0;
   m.activeLoop = !useExitPath && pro.motionPathLoop;
   if (!m.runtimePrepared)
   {
      m.startW = m.w;
      m.startH = m.h;
      if (m.sourceAnchor && m.sourceBounds.IsEmpty())
         m.sourceBounds = Rect(bulb); // Member 1 is also the feeder artwork: keep its start rectangle
      // Authored coordinates: the runtime routes are the authored ones, moved with the picture's start
      const float originalW = std::max(1.f, static_cast<float>(bulb.m_width)), originalH = std::max(1.f, static_cast<float>(bulb.m_height));
      const float scaleX = m.w / originalW, scaleY = m.h / originalH;
      const ProPoint originalCenter { bulb.m_locationX + originalW / 2.f, bulb.m_locationY + originalH / 2.f };
      const ProPoint runtimeCenter = Center(bulb);
      const auto map = [&](ProPoint p) { return ProPoint { runtimeCenter.x + (p.x - originalCenter.x) * scaleX, runtimeCenter.y + (p.y - originalCenter.y) * scaleY }; };
      m.entryPoints.clear();
      for (const ProPoint& p : m.authoredPoints)
         m.entryPoints.push_back(map(p));
      m.exitPoints.clear();
      for (const ProPoint& p : pro.motionPathExitPoints)
         m.exitPoints.push_back(map(p));
      m.respawnPoint = map(pro.motionPathRespawnPoint);
      m.runtimePrepared = true;
   }
   m.runtimePoints = useExitPath ? m.exitPoints : m.entryPoints;
   if (!useExitPath && startFromCurrentPosition)
   {
      vector<ProPoint> route { Center(bulb) };
      for (const ProPoint& p : m.entryPoints)
      {
         const float dx = p.x - route.back().x, dy = p.y - route.back().y;
         if (dx * dx + dy * dy > 0.0001f)
            route.push_back(p);
      }
      m.runtimePoints = route;
   }
   if (m.runtimePoints.size() < 2)
      return;
   if (!useExitPath)
      m.arrivalSpeed = static_cast<float>(PathLength(m.runtimePoints) / std::max(1, m.activeDuration));
   m.launchReported = useExitPath;
   m.launchProgress = useExitPath ? 1.f : FirstSegmentCompletion(m.runtimePoints);
   SetCenter(bulb, m.runtimePoints[0], false);
   bulb.m_pro->rt.visible = true;
   m.clockMs = 0.0;
   m.timerOn = true;
}

void B2SProMotionSystem::StartShift(B2SBulb& bulb, ProPoint target, int duration, int delayMs)
{
   ProMotion& m = M(bulb);
   StopCollision(bulb, true);
   m.timerOn = false;
   m.clockMs = 0.0;
   m.paused = false;
   m.externalShift = false;
   m.launchReported = true;
   m.startW = m.w;
   m.startH = m.h;
   m.runtimePoints = { Center(bulb), target };
   m.activeDuration = std::clamp(duration, 250, 30000);
   m.startDelay = std::max(0, delayMs);
   m.activeLoop = false;
   bulb.m_pro->rt.visible = true;
   m.timerOn = true;
}

static ProPoint GridPoint(const ProMotion& m, float progress)
{
   const ProPoint p0 = m.runtimePoints[0], p1 = m.runtimePoints[1];
   const float t = std::clamp(progress, 0.f, 1.f), t2 = t * t, t3 = t2 * t;
   const float duration = static_cast<float>(std::max(1, m.activeDuration));
   const float h00 = 2.f * t3 - 3.f * t2 + 1.f, h10 = t3 - 2.f * t2 + t, h01 = -2.f * t3 + 3.f * t2, h11 = t3 - t2;
   return { h00 * p0.x + h10 * duration * m.extStartVel.x + h01 * p1.x + h11 * duration * m.extEndVel.x,
      h00 * p0.y + h10 * duration * m.extStartVel.y + h01 * p1.y + h11 * duration * m.extEndVel.y };
}

static ProPoint GridVelocity(const ProMotion& m, float progress)
{
   const ProPoint p0 = m.runtimePoints[0], p1 = m.runtimePoints[1];
   const float t = std::clamp(progress, 0.f, 1.f), t2 = t * t;
   const float duration = static_cast<float>(std::max(1, m.activeDuration));
   const float dh00 = 6.f * t2 - 6.f * t, dh10 = 3.f * t2 - 4.f * t + 1.f, dh01 = -6.f * t2 + 6.f * t, dh11 = 3.f * t2 - 2.f * t;
   return { (dh00 * p0.x + dh10 * duration * m.extStartVel.x + dh01 * p1.x + dh11 * duration * m.extEndVel.x) / duration,
      (dh00 * p0.y + dh10 * duration * m.extStartVel.y + dh01 * p1.y + dh11 * duration * m.extEndVel.y) / duration };
}

void B2SProMotionSystem::StartExternalGridShift(B2SBulb& bulb, ProPoint target, int duration, ProPoint lookAhead)
{
   ProMotion& m = M(bulb);
   ProPoint startCenter = Center(bulb);
   ProPoint startVelocity = m.extLastVel;
   if (m.externalShift && m.timerOn && m.runtimePoints.size() >= 2)
   {
      const float oldProgress = static_cast<float>(std::min(1.0, m.clockMs / std::max(1, m.activeDuration)));
      startCenter = GridPoint(m, oldProgress);
      startVelocity = GridVelocity(m, oldProgress);
      SetCenter(bulb, startCenter, true);
   }
   m.timerOn = false;
   m.clockMs = 0.0;
   m.paused = false;
   m.launchReported = true;
   m.startW = m.w;
   m.startH = m.h;
   m.runtimePoints = { startCenter, target };
   m.activeDuration = std::clamp(duration, 10, 1000);
   m.startDelay = 0;
   m.activeLoop = false;
   m.externalShift = true;
   // LimitExternalGridVelocity
   {
      const double dx = target.x - startCenter.x, dy = target.y - startCenter.y;
      const double distance = std::sqrt(dx * dx + dy * dy);
      const double speed = std::sqrt(static_cast<double>(startVelocity.x) * startVelocity.x + static_cast<double>(startVelocity.y) * startVelocity.y);
      const double maximum = std::max(0.01, distance * 2.0 / std::max(1, m.activeDuration));
      if (speed <= maximum || speed <= 0.0)
         m.extStartVel = startVelocity;
      else
      {
         const float scale = static_cast<float>(maximum / speed);
         m.extStartVel = { startVelocity.x * scale, startVelocity.y * scale };
      }
   }
   m.extEndVel = { (lookAhead.x - target.x) / m.activeDuration, (lookAhead.y - target.y) / m.activeDuration };
   m.extLastVel = m.extStartVel;
   bulb.m_pro->rt.visible = true;
   m.timerOn = true;
}

void B2SProMotionSystem::StartDrainAll(B2SBulb& bulb, const vector<ProPoint>& route, int duration)
{
   if (route.size() < 2)
      return;
   ProMotion& m = M(bulb);
   StopCollision(bulb, true);
   m.timerOn = false;
   m.clockMs = 0.0;
   m.paused = false;
   m.externalShift = false;
   m.startW = m.w;
   m.startH = m.h;
   m.runtimePoints = route;
   m.activeDuration = std::clamp(duration, 250, 30000);
   m.startDelay = 0;
   m.activeLoop = false;
   m.launchReported = true;
   SetCenter(bulb, m.runtimePoints[0], false);
   bulb.m_pro->rt.visible = true;
   m.timerOn = true;
}

vector<ProPoint> B2SProMotionSystem::ExitRuntimeRoute(const B2SBulb& bulb) const
{
   vector<ProPoint> route;
   const vector<ProPoint>& exit = bulb.m_pro->motionPathExitPoints;
   if (exit.size() < 2)
      return route;
   const ProMotion& m = M(bulb);
   const ProPoint start = Center(bulb);
   const float scaleX = bulb.m_width > 0 ? m.w / bulb.m_width : 1.f, scaleY = bulb.m_height > 0 ? m.h / bulb.m_height : 1.f;
   route.push_back(start);
   for (size_t i = 1; i < exit.size(); i++)
      route.push_back({ start.x + (exit[i].x - exit[0].x) * scaleX, start.y + (exit[i].y - exit[0].y) * scaleY });
   return route;
}

void B2SProMotionSystem::SetMotionPathPosition(B2SBulb& bulb, ProPoint center, bool advanceRoll)
{
   ProMotion& m = M(bulb);
   StopCollision(bulb, true);
   m.timerOn = false;
   m.clockMs = 0.0;
   m.paused = false;
   m.activeLoop = false;
   m.startDelay = 0;
   m.launchReported = true;
   m.startW = m.w;
   m.startH = m.h;
   SetCenter(bulb, center, advanceRoll);
}

void B2SProMotionSystem::StopMotionPath(B2SBulb& bulb)
{
   ProMotion& m = M(bulb);
   if (m.timerOn)
   {
      m.timerOn = false;
      m.paused = true; // The clock keeps its elapsed time
   }
}

void B2SProMotionSystem::ResumeMotionPath(B2SBulb& bulb)
{
   ProMotion& m = M(bulb);
   if (!m.paused || m.runtimePoints.size() < 2)
      return;
   m.paused = false;
   m.timerOn = true;
}

void B2SProMotionSystem::SetSourceAnchor(B2SBulb& bulb, bool enabled)
{
   ProMotion& m = M(bulb);
   if (m.sourceAnchor == enabled)
      return;
   m.sourceAnchor = enabled;
   m.sourceVisible = enabled;
   if (!enabled)
   {
      StopSourceRespawn(bulb);
      m.sourceBounds = {};
      m.sourceImage = nullptr;
   }
}

void B2SProMotionSystem::SetSourceImage(B2SBulb& bulb, VPXTexture image) { M(bulb).sourceImage = image; }

void B2SProMotionSystem::SetSourceVisible(B2SBulb& bulb, bool visible)
{
   ProMotion& m = M(bulb);
   if (!m.sourceAnchor || m.sourceVisible == visible)
      return;
   if (!visible)
      StopSourceRespawn(bulb);
   m.sourceVisible = visible;
}

bool B2SProMotionSystem::StartSourceRespawn(B2SBulb& bulb)
{
   ProMotion& m = M(bulb);
   if (!m.sourceAnchor || !bulb.m_pro->motionPathRespawnEnabled || !m.runtimePrepared || m.sourceBounds.IsEmpty())
      return false;
   const float w = m.sourceBounds.w, h = m.sourceBounds.h;
   const ProRect start { m.respawnPoint.x - w / 2.f, m.respawnPoint.y - h / 2.f, w, h };
   if (std::abs(start.x - m.sourceBounds.x) < 0.5f && std::abs(start.y - m.sourceBounds.y) < 0.5f)
      return false;
   StopSourceRespawn(bulb);
   m.respawnStart = start;
   m.respawnEnd = m.sourceBounds;
   m.respawnBounds = start;
   m.respawnActive = true;
   m.sourceVisible = true;
   m.respawnClockMs = 0.0;
   return true;
}

void B2SProMotionSystem::StopSourceRespawn(B2SBulb& bulb)
{
   ProMotion& m = M(bulb);
   m.respawnClockMs = 0.0;
   m.respawnActive = false;
   m.respawnBounds = {};
}

bool B2SProMotionSystem::IsSourceSeparate(const B2SBulb& bulb)
{
   if (!bulb.m_pro)
      return false;
   const ProMotion& m = M(bulb);
   if (!(m.sourceAnchor && m.authoredPoints.size() >= 2) || !m.sourceVisible)
      return false;
   const ProRect source = SourceRect(bulb);
   return !bulb.m_pro->rt.visible || std::abs(source.x - m.x) > 0.5f || std::abs(source.y - m.y) > 0.5f || std::abs(source.w - m.w) > 0.5f || std::abs(source.h - m.h) > 0.5f;
}

ProRect B2SProMotionSystem::SourceRect(const B2SBulb& bulb)
{
   const ProMotion& m = M(bulb);
   if (m.respawnActive)
      return m.respawnBounds;
   if (!m.sourceBounds.IsEmpty())
      return m.sourceBounds;
   return Rect(bulb);
}

VPXTexture B2SProMotionSystem::SourceImage(const B2SBulb& bulb)
{
   const ProMotion& m = M(bulb);
   return m.sourceImage ? m.sourceImage : m.image;
}

float B2SProMotionSystem::SourceRollAngle(const B2SBulb& bulb) { return bulb.m_pro->motionPathRollEnabled ? M(bulb).sourceRollAngle : 0.f; }

void B2SProMotionSystem::Tick(B2SBulb& bulb, double elapsedMs)
{
   ProMotion& m = M(bulb);
   if (m.timerOn)
   {
      // MotionPathTimer_Tick
      if (m.runtimePoints.size() < 2)
      {
         m.timerOn = false;
         m.clockMs = 0.0;
      }
      else
      {
         m.clockMs += elapsedMs;
         if (m.clockMs >= m.startDelay)
         {
            const float progress = static_cast<float>(std::min(1.0, (m.clockMs - m.startDelay) / m.activeDuration));
            if (m.externalShift)
            {
               SetCenter(bulb, GridPoint(m, progress), true);
               m.extLastVel = GridVelocity(m, progress);
            }
            else
            {
               // PointAlongMotionPath
               const vector<ProPoint>& pts = m.runtimePoints;
               const size_t segmentCount = pts.size() - 1 + (m.activeLoop ? 1 : 0);
               vector<double> lengths;
               double total = 0.0;
               for (size_t i = 0; i < segmentCount; i++)
               {
                  const ProPoint a = pts[i % pts.size()], b = pts[(i + 1) % pts.size()];
                  const double dx = b.x - a.x, dy = b.y - a.y;
                  lengths.push_back(std::sqrt(dx * dx + dy * dy));
                  total += lengths.back();
               }
               ProPoint point = pts[0];
               if (total > 0.0)
               {
                  double target = std::clamp(static_cast<double>(progress), 0.0, 1.0) * total;
                  point = pts.back();
                  for (size_t i = 0; i < lengths.size(); i++)
                  {
                     if (target <= lengths[i] || i == lengths.size() - 1)
                     {
                        const float part = lengths[i] <= 0.0 ? 0.f : static_cast<float>(target / lengths[i]);
                        const ProPoint a = pts[i % pts.size()], b = pts[(i + 1) % pts.size()];
                        point = { a.x + (b.x - a.x) * part, a.y + (b.y - a.y) * part };
                        break;
                     }
                     target -= lengths[i];
                  }
               }
               SetCenter(bulb, point, true);
               if (!m.launchReported && progress >= m.launchProgress)
               {
                  m.launchReported = true;
                  OnLaunchSegmentCompleted(bulb);
               }
            }
            if (progress >= 1.f)
            {
               if (m.activeLoop)
                  m.clockMs = 0.0;
               else
               {
                  m.timerOn = false;
                  m.clockMs = 0.0;
                  m.paused = false;
                  OnCompleted(bulb);
                  if (bulb.m_pro->motionPathQueueTriggers && m.queuedStarts > 0)
                  {
                     m.queuedStarts--;
                     StartMotionPath(bulb);
                  }
               }
            }
         }
      }
   }

   if (m.collisionOn)
   {
      // TroughCollisionTimer_Tick
      m.collisionClockMs += elapsedMs;
      if (m.collisionClockMs >= m.collisionDelay)
      {
         const float progress = static_cast<float>((m.collisionClockMs - m.collisionDelay) / 180.0);
         if (progress >= 1.f)
            StopCollision(bulb, true);
         else
         {
            const float offset = static_cast<float>(m.collisionAmp * std::sin(kPi * progress) * std::exp(-1.2 * progress));
            const ProPoint base = RectCenter(m.collisionBase);
            const ProPoint center { base.x + m.collisionDir.x * offset, base.y + m.collisionDir.y * offset };
            AdvanceRoll(bulb, Center(bulb), center, m.collisionBase.w, m.collisionBase.h, m.rollAngle);
            m.x = center.x - m.collisionBase.w / 2.f;
            m.y = center.y - m.collisionBase.h / 2.f;
            m.w = m.collisionBase.w;
            m.h = m.collisionBase.h;
         }
      }
   }

   if (m.respawnActive)
   {
      // MotionPathSourceRespawnTimer_Tick
      m.respawnClockMs += elapsedMs;
      const int duration = std::clamp(bulb.m_pro->motionPathRespawnDuration, 50, 5000);
      const float progress = static_cast<float>(std::min(1.0, m.respawnClockMs / duration));
      const float eased = 1.f - std::pow(1.f - progress, 3.f);
      const ProRect next { m.respawnStart.x + (m.respawnEnd.x - m.respawnStart.x) * eased, m.respawnStart.y + (m.respawnEnd.y - m.respawnStart.y) * eased, m.respawnEnd.w, m.respawnEnd.h };
      AdvanceRoll(bulb, RectCenter(m.respawnBounds), RectCenter(next), m.respawnEnd.w, m.respawnEnd.h, m.sourceRollAngle);
      m.respawnBounds = next;
      if (progress >= 1.f)
      {
         m.respawnBounds = m.respawnEnd;
         m.respawnClockMs = 0.0;
         m.respawnActive = false;
         OnSourceRespawnCompleted(bulb);
      }
   }
}

// ------------------------------------------------------------------------------------------------
// Events

void B2SProMotionSystem::OnCompleted(B2SBulb& bulb)
{
   ProMotion& m = M(bulb);
   if (m.sequence < 0)
      return;
   Sequence& s = m_sequences[static_cast<size_t>(m.sequence)];
   const bool entry = m.subEntry, removal = m.subRemoval, drain = m.subDrain;
   if (entry)
      SequenceEntryCompleted(s, &bulb);
   if (removal)
      SequenceRemovalCompleted(s, &bulb);
   if (drain)
      SequenceDrainAllRemovalCompleted(s, &bulb);
}

void B2SProMotionSystem::OnLaunchSegmentCompleted(B2SBulb& bulb)
{
   ProMotion& m = M(bulb);
   if (m.sequence < 0 || !m.subLaunch)
      return;
   m.subLaunch = false;
   Sequence& s = m_sequences[static_cast<size_t>(m.sequence)];
   if (&bulb == s.launchingMember && Contains(s.activeEntries, &bulb))
      SequenceBeginSourceRespawn(s);
}

void B2SProMotionSystem::OnSourceRespawnCompleted(B2SBulb& bulb)
{
   ProMotion& m = M(bulb);
   if (m.sequence < 0 || !m.subRespawn)
      return;
   m.subRespawn = false;
   SequenceCompleteSourceRespawn(m_sequences[static_cast<size_t>(m.sequence)]);
}

// ------------------------------------------------------------------------------------------------
// B2SData.MotionPathSequenceState

void B2SProMotionSystem::StartMotionPaths(const vector<B2SBulb*>& bulbs)
{
   vector<size_t> requested;
   for (B2SBulb* bulb : bulbs)
   {
      const ProMotion& m = M(*bulb);
      if (m.sequence >= 0 && m.sequenceOrder > 0)
      {
         if (std::find(requested.begin(), requested.end(), static_cast<size_t>(m.sequence)) == requested.end())
            requested.push_back(static_cast<size_t>(m.sequence));
      }
      else
         StartMotionPath(*bulb);
   }
   for (size_t index : requested)
      SequenceRequestStart(m_sequences[index]);
}

void B2SProMotionSystem::RemoveMotionPaths(const vector<B2SBulb*>& bulbs)
{
   vector<size_t> requested;
   for (B2SBulb* bulb : bulbs)
   {
      const ProMotion& m = M(*bulb);
      if (m.sequence >= 0 && m.sequenceOrder > 0)
      {
         if (std::find(requested.begin(), requested.end(), static_cast<size_t>(m.sequence)) == requested.end())
            requested.push_back(static_cast<size_t>(m.sequence));
      }
      else if (bulb->m_pro->motionPathExitPoints.size() >= 2)
         StartMotionPathExit(*bulb);
   }
   for (size_t index : requested)
      SequenceRequestRemove(m_sequences[index]);
}

static string MemberName(const B2SBulb* b) { return "picturebox" + std::to_string(b->m_pro->id); }

void B2SProMotionSystem::SequenceRegister(Sequence& s, B2SBulb* bulb)
{
   if (Contains(s.members, bulb))
      return;
   ProMotion& m = M(*bulb);
   if (bulb->m_pro->motionPathRollEnabled)
      m.rollAngle = static_cast<float>(std::fmod((std::max(1, m.sequenceOrder) - 1) * 137.507764, 360.0)); // Golden angle phase
   s.members.push_back(bulb);
   std::stable_sort(s.members.begin(), s.members.end(),
      [](const B2SBulb* a, const B2SBulb* b)
      {
         const int oa = M(*a).sequenceOrder, ob = M(*b).sequenceOrder;
         if (oa != ob)
            return oa < ob;
         return MemberName(a) < MemberName(b);
      });
   for (B2SBulb* member : s.members)
      SetSourceAnchor(*member, false);
   s.sourceAnchor = s.members.empty() ? nullptr : s.members.front();
   if (s.sourceAnchor)
   {
      SetSourceAnchor(*s.sourceAnchor, true);
      SequenceUpdateSourceAppearance(s);
      SetSourceVisible(*s.sourceAnchor, s.sourceReady);
   }
}

bool B2SProMotionSystem::SequenceQueueEnabled(const Sequence& s) const
{
   return std::any_of(s.members.begin(), s.members.end(), [](const B2SBulb* b) { return b->m_pro->motionPathQueueTriggers; });
}
bool B2SProMotionSystem::SequenceDrainAllEnabled(const Sequence& s) const
{
   return std::any_of(s.members.begin(), s.members.end(), [](const B2SBulb* b) { return b->m_pro->motionPathDrainAll; });
}
bool B2SProMotionSystem::SequenceGravityDropEnabled(const Sequence& s) const
{
   return std::any_of(s.members.begin(), s.members.end(), [](const B2SBulb* b) { return b->m_pro->motionPathGravityDrop; });
}

void B2SProMotionSystem::SequenceQueue(Sequence& s, bool remove)
{
   if (SequenceQueueEnabled(s) && s.pendingOperations.size() < 32)
      s.pendingOperations.push_back(remove);
}

void B2SProMotionSystem::SequenceRequestStart(Sequence& s)
{
   if (!s.pendingOperations.empty())
   {
      SequenceQueue(s, false);
      SequenceProcessPending(s);
      return;
   }
   if (s.activeRemoval != nullptr || !s.activeDrainAllRemovals.empty())
   {
      SequenceQueue(s, false);
      return;
   }
   if (!s.sourceReady)
   {
      if (SequenceQueueEnabled(s))
         SequenceQueue(s, false);
      else if (s.launchingMember != nullptr && Contains(s.activeEntries, s.launchingMember))
         SequenceBeginEntry(s, s.launchingMember, false);
      return;
   }
   if (!SequenceStartEntry(s) && SequenceGravityDropEnabled(s) && s.gravityReuseEnabled && !s.availableGravityMembers.empty() && SequenceQueueEnabled(s))
      SequenceQueue(s, false);
}

void B2SProMotionSystem::SequenceRequestRemove(Sequence& s)
{
   if (!s.pendingOperations.empty() || !s.activeEntries.empty())
   {
      SequenceQueue(s, true);
      SequenceProcessPending(s);
      return;
   }
   if (!s.activeDrainAllRemovals.empty())
   {
      if (SequenceQueueEnabled(s))
         SequenceQueue(s, true);
      return;
   }
   if (s.activeRemoval != nullptr)
   {
      if (SequenceQueueEnabled(s))
         SequenceQueue(s, true);
      else
         StartMotionPathExit(*s.activeRemoval);
      return;
   }
   if (SequenceGravityDropEnabled(s))
      SequenceStartGravityDrop(s);
   else if (SequenceDrainAllEnabled(s))
      SequenceStartAllRemovals(s);
   else
      SequenceStartRemoval(s);
}

bool B2SProMotionSystem::SequenceStartEntry(Sequence& s)
{
   static std::mt19937 gravityReturnRandom { std::random_device {}() };
   if (!s.sourceReady || s.activeRemoval != nullptr || !s.activeDrainAllRemovals.empty())
      return false;
   B2SBulb* memberToStart = nullptr;
   bool startFromCurrentPosition = false;
   if (SequenceGravityDropEnabled(s) && s.gravityReuseEnabled)
   {
      vector<B2SBulb*> available;
      for (B2SBulb* member : s.availableGravityMembers)
         if (!Contains(s.occupied, member) && !Contains(s.activeEntries, member))
            available.push_back(member);
      if (!available.empty())
      {
         memberToStart = available[std::uniform_int_distribution<size_t>(0, available.size() - 1)(gravityReturnRandom)];
         startFromCurrentPosition = true;
      }
      if (memberToStart != nullptr)
      {
         Remove(s.availableGravityMembers, memberToStart);
         SequenceAssignNextOpenEntrySlot(s, memberToStart);
      }
   }
   if (memberToStart == nullptr)
      for (B2SBulb* member : s.members)
         if ((memberToStart == nullptr || M(*member).sequenceOrder < M(*memberToStart).sequenceOrder) && !Contains(s.occupied, member) && !Contains(s.activeEntries, member))
            memberToStart = member;
   if (memberToStart == nullptr)
      return false;
   deactivateGravityDrop(*memberToStart);
   AddUnique(s.activeEntries, memberToStart);
   M(*memberToStart).subEntry = true;
   SequenceBeginEntry(s, memberToStart, startFromCurrentPosition);
   return true;
}

void B2SProMotionSystem::SequenceAssignNextOpenEntrySlot(Sequence& s, B2SBulb* captured)
{
   if (captured == nullptr)
      return;
   B2SBulb* slotOwner = nullptr;
   for (B2SBulb* member : s.members)
      if (!Contains(s.occupied, member) && !Contains(s.activeEntries, member) && (slotOwner == nullptr || M(*member).sequenceOrder < M(*slotOwner).sequenceOrder))
         slotOwner = member;
   if (slotOwner == nullptr || slotOwner == captured)
      return;
   // SwapMotionPathEntryRouteWith: authored and runtime entry routes, then the orders
   ProMotion& a = M(*captured);
   ProMotion& b = M(*slotOwner);
   std::swap(a.authoredPoints, b.authoredPoints);
   std::swap(a.entryPoints, b.entryPoints);
   std::swap(a.sequenceOrder, b.sequenceOrder);
}

bool B2SProMotionSystem::SequenceStartRemoval(Sequence& s)
{
   if (s.activeRemoval != nullptr || !s.activeDrainAllRemovals.empty() || !s.activeEntries.empty())
      return false;
   for (B2SBulb* member : s.members)
      if (Contains(s.occupied, member) && member->m_pro->motionPathExitPoints.size() >= 2)
      {
         s.activeRemoval = member;
         break;
      }
   if (s.activeRemoval == nullptr)
      return false;
   M(*s.activeRemoval).subRemoval = true;
   SequenceStartCompactionShifts(s, s.activeRemoval);
   StartMotionPathExit(*s.activeRemoval);
   return true;
}

bool B2SProMotionSystem::SequenceStartAllRemovals(Sequence& s)
{
   if (s.activeRemoval != nullptr || !s.activeDrainAllRemovals.empty() || !s.activeEntries.empty())
      return false;
   vector<B2SBulb*> removals;
   for (B2SBulb* member : s.members)
      if (Contains(s.occupied, member) && member->m_pro->motionPathExitPoints.size() >= 2)
         removals.push_back(member);
   if (removals.empty())
      return false;
   std::stable_sort(removals.begin(), removals.end(), [](const B2SBulb* a, const B2SBulb* b) { return M(*a).sequenceOrder < M(*b).sequenceOrder; });
   for (B2SBulb* member : removals)
   {
      AddUnique(s.activeDrainAllRemovals, member);
      M(*member).subDrain = true;
   }
   B2SBulb* lead = removals.front();
   const vector<ProPoint> leadRoute = ExitRuntimeRoute(*lead);
   const double leadLength = PathLength(leadRoute);
   const double leadSpeed = leadLength > 0.001 ? leadLength / std::max(250, lead->m_pro->motionPathExitDuration) : 0.0;
   if (leadSpeed <= 0.000001)
   {
      for (B2SBulb* member : removals)
         StartMotionPathExit(*member);
      return true;
   }
   for (size_t index = 0; index < removals.size(); index++)
   {
      B2SBulb* member = removals[index];
      vector<ProPoint> trainRoute { Center(*member) };
      for (size_t preceding = index; preceding-- > 0;)
         AddDistinct(trainRoute, Center(*removals[preceding]));
      for (size_t r = 1; r < leadRoute.size(); r++)
         AddDistinct(trainRoute, leadRoute[r]);
      const int memberDuration = static_cast<int>(std::nearbyint(PathLength(trainRoute) / leadSpeed));
      StartDrainAll(*member, trainRoute, memberDuration);
   }
   return true;
}

bool B2SProMotionSystem::SequenceStartGravityDrop(Sequence& s)
{
   if (s.activeRemoval != nullptr || !s.activeDrainAllRemovals.empty() || !s.activeEntries.empty())
      return false;
   vector<B2SBulb*> released;
   s.gravityReuseEnabled = true;
   for (B2SBulb* member : s.members)
      if (Contains(s.occupied, member) && activateGravityDrop(*member))
      {
         released.push_back(member);
         AddUnique(s.availableGravityMembers, member);
      }
   if (released.empty())
   {
      if (s.availableGravityMembers.empty())
      {
         s.gravityReuseEnabled = false;
         return false;
      }
      s.compactionTargets.clear();
      SequenceUpdateSourceAppearance(s);
      SequenceProcessPending(s);
      return true;
   }
   for (B2SBulb* member : released)
      Remove(s.occupied, member);
   s.compactionTargets.clear();
   SequenceUpdateSourceAppearance(s);
   SequenceProcessPending(s);
   return true;
}

void B2SProMotionSystem::SequenceBeginEntry(Sequence& s, B2SBulb* member, bool startFromCurrentPosition)
{
   if (member == nullptr)
      return;
   s.launchingMember = member;
   s.sourceReady = false;
   if (s.sourceAnchor != nullptr)
   {
      SetSourceImage(*s.sourceAnchor, M(*member).image);
      SetSourceVisible(*s.sourceAnchor, false);
   }
   M(*member).subLaunch = true;
   if (startFromCurrentPosition)
      StartMotionPathFromCurrentPosition(*member);
   else
      StartMotionPath(*member);
}

void B2SProMotionSystem::SequenceBeginSourceRespawn(Sequence& s)
{
   s.launchingMember = nullptr;
   if (s.sourceAnchor != nullptr)
   {
      SequenceUpdateSourceAppearance(s);
      M(*s.sourceAnchor).subRespawn = true;
      if (StartSourceRespawn(*s.sourceAnchor))
         return;
      M(*s.sourceAnchor).subRespawn = false;
   }
   SequenceCompleteSourceRespawn(s);
}

void B2SProMotionSystem::SequenceCompleteSourceRespawn(Sequence& s)
{
   s.sourceReady = true;
   if (s.sourceAnchor != nullptr)
      SetSourceVisible(*s.sourceAnchor, true);
   SequenceProcessPending(s);
}

void B2SProMotionSystem::SequenceProcessPending(Sequence& s)
{
   while (!s.pendingOperations.empty())
   {
      const bool remove = s.pendingOperations.front();
      if (remove)
      {
         if (s.activeRemoval != nullptr || !s.activeDrainAllRemovals.empty() || !s.activeEntries.empty())
            return;
         s.pendingOperations.erase(s.pendingOperations.begin());
         const bool started = SequenceGravityDropEnabled(s) ? SequenceStartGravityDrop(s) : (SequenceDrainAllEnabled(s) ? SequenceStartAllRemovals(s) : SequenceStartRemoval(s));
         if (started)
            return;
      }
      else
      {
         if (s.activeRemoval != nullptr || !s.activeDrainAllRemovals.empty() || !s.sourceReady)
            return;
         if (SequenceStartEntry(s))
            s.pendingOperations.erase(s.pendingOperations.begin());
         return;
      }
   }
}

void B2SProMotionSystem::SequenceStartCompactionShifts(Sequence& s, B2SBulb* removing)
{
   s.compactionTargets.clear();
   vector<B2SBulb*> occupiedInOrder;
   for (B2SBulb* member : s.members)
      if (Contains(s.occupied, member))
         occupiedInOrder.push_back(member);
   if (occupiedInOrder.empty() || occupiedInOrder.front() != removing)
      return;
   for (size_t index = 0; index + 1 < occupiedInOrder.size(); index++)
   {
      s.compactionTargets.push_back(Center(*occupiedInOrder[index]));
      StartShift(*occupiedInOrder[index + 1], s.compactionTargets[index], removing->m_pro->motionPathExitDuration, static_cast<int>(index) * 48);
   }
}

void B2SProMotionSystem::SequenceNormalizeCompactedSlots(Sequence& s, int remainingCount)
{
   // The survivors already rolled into their new slots; the lower numbered members take their place
   // (images and roll angles carried across) so the next entry uses the right open slot
   vector<VPXTexture> survivorImages, unusedImages;
   vector<float> survivorRoll, unusedRoll;
   for (B2SBulb* member : s.members)
   {
      if (Contains(s.occupied, member))
      {
         survivorImages.push_back(M(*member).image);
         survivorRoll.push_back(M(*member).rollAngle);
      }
      else
      {
         unusedImages.push_back(M(*member).image);
         unusedRoll.push_back(M(*member).rollAngle);
      }
   }
   s.occupied.clear();
   for (int index = 0; index < static_cast<int>(s.members.size()); index++)
   {
      B2SBulb* member = s.members[static_cast<size_t>(index)];
      if (index < remainingCount && index < static_cast<int>(survivorRoll.size()))
      {
         M(*member).image = survivorImages[static_cast<size_t>(index)];
         M(*member).rollAngle = survivorRoll[static_cast<size_t>(index)];
      }
      else
      {
         const int unusedIndex = index - remainingCount;
         if (unusedIndex >= 0 && unusedIndex < static_cast<int>(unusedRoll.size()))
         {
            M(*member).image = unusedImages[static_cast<size_t>(unusedIndex)];
            M(*member).rollAngle = unusedRoll[static_cast<size_t>(unusedIndex)];
         }
      }
      SetMotionPathPosition(*member, Center(*member));
      if (index < remainingCount)
      {
         if (index < static_cast<int>(s.compactionTargets.size()))
            SetMotionPathPosition(*member, s.compactionTargets[static_cast<size_t>(index)]);
         member->m_pro->rt.visible = true;
         AddUnique(s.occupied, member);
      }
      else
         member->m_pro->rt.visible = false;
   }
   s.compactionTargets.clear();
   SequenceUpdateSourceAppearance(s);
}

void B2SProMotionSystem::SequenceUpdateSourceAppearance(Sequence& s)
{
   if (s.sourceAnchor == nullptr)
      return;
   for (B2SBulb* member : s.members)
      if (!Contains(s.occupied, member) && !Contains(s.activeEntries, member) && member != s.activeRemoval)
      {
         SetSourceImage(*s.sourceAnchor, M(*member).image);
         return;
      }
}

void B2SProMotionSystem::SequenceEntryCompleted(Sequence& s, B2SBulb* member)
{
   M(*member).subEntry = false;
   M(*member).subLaunch = false;
   if (Contains(s.activeEntries, member))
   {
      Remove(s.activeEntries, member);
      SequenceApplyEntryCollision(s, member);
      AddUnique(s.occupied, member);
      if (member == s.launchingMember)
         SequenceBeginSourceRespawn(s);
   }
   SequenceProcessPending(s);
}

void B2SProMotionSystem::SequenceApplyEntryCollision(Sequence& s, B2SBulb* incoming)
{
   if (incoming == nullptr || s.occupied.empty())
      return;
   vector<B2SBulb*> impacted;
   for (B2SBulb* member : s.members)
      if (Contains(s.occupied, member))
         impacted.push_back(member);
   std::stable_sort(impacted.begin(), impacted.end(), [](const B2SBulb* a, const B2SBulb* b) { return M(*b).sequenceOrder < M(*a).sequenceOrder; });
   if (impacted.empty())
      return;
   const ProPoint incomingCenter = Center(*incoming);
   const ProPoint nearestCenter = Center(*impacted.front());
   ProPoint direction { nearestCenter.x - incomingCenter.x, nearestCenter.y - incomingCenter.y };
   const double length = std::sqrt(static_cast<double>(direction.x) * direction.x + static_cast<double>(direction.y) * direction.y);
   if (length <= 0.001)
      return;
   direction = { static_cast<float>(direction.x / length), static_cast<float>(direction.y / length) };
   const ProMotion& im = M(*incoming);
   const float ballDiameter = std::max(1.f, std::min(im.w, im.h));
   float impactAmplitude = im.arrivalSpeed * 4.5f;
   impactAmplitude = std::max(ballDiameter * 0.012f, std::min(ballDiameter * 0.04f, impactAmplitude)) * 30.f;
   StartCollision(*incoming, { -direction.x, -direction.y }, impactAmplitude * 0.35f, 0);
   float transferred = impactAmplitude;
   for (size_t index = 0; index < impacted.size(); index++)
   {
      StartCollision(*impacted[index], direction, transferred, static_cast<int>(index) * 18);
      transferred *= 0.58f;
      if (transferred < 0.25f)
         break;
   }
}

void B2SProMotionSystem::SequenceRemovalCompleted(Sequence& s, B2SBulb* member)
{
   M(*member).subRemoval = false;
   if (member == s.activeRemoval)
   {
      Remove(s.occupied, member);
      member->m_pro->rt.visible = false;
      SequenceNormalizeCompactedSlots(s, static_cast<int>(s.occupied.size()));
      s.activeRemoval = nullptr;
   }
   SequenceProcessPending(s);
}

void B2SProMotionSystem::SequenceDrainAllRemovalCompleted(Sequence& s, B2SBulb* member)
{
   M(*member).subDrain = false;
   if (!Contains(s.activeDrainAllRemovals, member))
      return;
   Remove(s.activeDrainAllRemovals, member);
   Remove(s.occupied, member);
   member->m_pro->rt.visible = false;
   if (s.activeDrainAllRemovals.empty())
   {
      s.compactionTargets.clear();
      SequenceUpdateSourceAppearance(s);
      SequenceProcessPending(s);
   }
}

}
