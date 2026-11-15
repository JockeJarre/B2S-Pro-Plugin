// license:GPLv3+

// B2S Pro: native one-image rotation, pivots, external pivots and named pivot commands.
// Port of B2SPictureBox (StartNativeRotation, StopNativeRotation, SetNativeRotationState,
// NativeRotationTimer_Tick, SetPivotTriggerState, SetPivotRotationTarget, PivotRotationTimer_Tick)
// and B2SData (RegisterPivotTrigger, RegisterSingleImagePivot, RegisterExternalPivot,
// ExternalPivotState, TryMoveExternalPivot) of the B2S Pro Server, with the Server's timers replaced
// by the frame time.

#include "B2SProRuntime.h"
#include "B2SProData.h"
#include "B2SServer.h"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace B2S {

static string TrimCopy(const string& s)
{
   size_t b = 0, e = s.size();
   while (b < e && std::isspace(static_cast<unsigned char>(s[b])))
      b++;
   while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])))
      e--;
   return s.substr(b, e - b);
}

string ProCommandKey(const string& name)
{
   string key = TrimCopy(name);
   std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
   return key;
}

vector<string> ProNamedCommandTargets(const B2STable& table)
{
   vector<string> targets;
   if (!table.m_unifiedLayerDeck)
      return targets;
   for (const auto* bulbs : { &table.m_backglassIlluminations, &table.m_dmdIlluminations })
      for (const auto& bulb : *bulbs)
      {
         if (!bulb->m_pro)
            continue;
         const B2SProBulbData& pro = *bulb->m_pro;
         const string name = TrimCopy(pro.name);
         if (!pro.externalGridName.empty() && !name.empty())
            targets.push_back(ProCommandKey(name));
         if (!pro.externalPivotGroup.empty() && !name.empty())
            targets.push_back(ProCommandKey(name));
         if (pro.pivotAnimation && !pro.pivotPhysicsEnabled && pro.pivotTriggerType == 0 && !name.empty() && !pro.pivotDownTrigger.empty() && !pro.pivotUpTrigger.empty())
         {
            targets.push_back(ProCommandKey(pro.pivotDownTrigger));
            targets.push_back(ProCommandKey(pro.pivotUpTrigger));
         }
      }
   return targets;
}

B2SProRuntime::B2SProRuntime(B2STable& table)
   : m_table(table)
{
   const auto registerExternalPivot = [this](const string& groupKey, std::unordered_map<string, size_t>& groups, B2SBulb* bulb, const string& targetKey, bool representative, float angle, int duration)
   {
      size_t index;
      if (const auto it = groups.find(groupKey); it != groups.end())
         index = it->second;
      else
      {
         index = m_pivotStates.size();
         m_pivotStates.emplace_back();
         groups[groupKey] = index;
      }
      ExternalPivotState& state = m_pivotStates[index];
      state.angles[targetKey] = angle;
      ProBulbRuntime& rt = bulb->m_pro->rt;
      rt.visible = false;
      if (representative)
      {
         state.representative = bulb;
         rt.pivotRotation = true;
         rt.nativeRotation = true;
         rt.angle = angle;
      }
      state.duration = std::clamp(duration, 10, 1000);
      m_pivotTargets[targetKey] = index;
   };

   std::unordered_map<string, size_t> externalGroups;
   for (auto* bulbs : { &table.m_backglassIlluminations, &table.m_dmdIlluminations })
      for (auto& bulbPtr : *bulbs)
      {
         B2SBulb* const bulb = bulbPtr.get();
         if (!bulb->m_pro)
            continue;
         B2SProBulbData& pro = *bulb->m_pro;
         ProBulbRuntime& rt = pro.rt;
         rt.nativeRotation = pro.nativeRotation;
         rt.autoStartRotation = pro.automaticRotationContinuous;
         rt.steps = std::max(2, pro.rotateSteps);
         rt.interval = std::max(10, pro.rotateInterval);
         const string name = TrimCopy(pro.name);

         // B2SData.RegisterExternalPivot
         if (!pro.externalPivotGroup.empty() && !name.empty())
            registerExternalPivot(ProCommandKey(pro.externalPivotGroup), externalGroups, bulb, ProCommandKey(name), pro.externalPivotRepresentative, pro.externalPivotAngle, pro.pivotDuration);

         if (pro.pivotAnimation && !pro.pivotPhysicsEnabled)
         {
            if (pro.pivotTriggerType == 0)
            {
               // B2SData.RegisterSingleImagePivot: its own state, the image moves to the Down or Up angle
               if (!name.empty() && !pro.pivotDownTrigger.empty() && !pro.pivotUpTrigger.empty())
               {
                  const string groupKey = ProCommandKey("Pivot:" + name);
                  const size_t index = m_pivotStates.size();
                  m_pivotStates.emplace_back();
                  externalGroups[groupKey] = index;
                  ExternalPivotState& state = m_pivotStates[index];
                  const string downKey = ProCommandKey(pro.pivotDownTrigger);
                  const string upKey = ProCommandKey(pro.pivotUpTrigger);
                  state.angles[downKey] = pro.pivotDownAngle;
                  state.angles[upKey] = pro.pivotUpAngle;
                  state.representative = bulb;
                  state.duration = std::clamp(pro.pivotDuration, 10, 1000);
                  rt.visible = false;
                  rt.pivotRotation = true;
                  rt.nativeRotation = true;
                  rt.angle = pro.pivotDownAngle; // Only the representative registration (Down) sets the angle
                  m_pivotTargets[downKey] = index;
                  m_pivotTargets[upKey] = index;
               }
            }
            else if (pro.pivotTriggerType == 4 || pro.pivotTriggerId > 0)
            {
               // B2SData.RegisterPivotTrigger
               rt.pivotRotation = true;
               rt.nativeRotation = true;
               rt.downAngle = pro.pivotDownAngle;
               rt.upAngle = pro.pivotUpAngle;
               rt.moveDuration = std::clamp(pro.pivotDuration, 10, 5000);
               rt.automaticOscillation = pro.pivotAutomaticOscillation || pro.pivotTriggerType == 4;
               rt.angle = rt.automaticOscillation ? 0.f : pro.pivotDownAngle;
               if (pro.pivotTriggerType == 4)
                  rt.startupPivot = true;
               else
               {
                  rt.triggerIsPivot = true;
                  rt.triggerId = pro.pivotTriggerId;
                  switch (pro.pivotTriggerType)
                  {
                  case 5: rt.triggerRomIdType = static_cast<int>(B2SRomIDType::Switch); break;
                  case 1: rt.triggerRomIdType = static_cast<int>(B2SRomIDType::Solenoid); break;
                  case 2: rt.triggerRomIdType = static_cast<int>(B2SRomIDType::Lamp); break;
                  default: rt.triggerFromScript = true; break; // PivotB2SIDs: B2SSetData lamp-id writes
                  }
               }
            }
         }
      }

   // Motion paths, sequences and position grids (registered while loading, before the startup images)
   m_motion = std::make_unique<B2SProMotionSystem>(table);
   // Physics balls (RegisterPhysicsBall runs while loading; a non trough ball starts and is shown)
   m_physics = std::make_unique<B2SProPhysics>(table, *m_motion);
   m_motion->activateGravityDrop = [this](B2SBulb& bulb) { return m_physics->ActivateGravityDrop(bulb); };
   m_motion->deactivateGravityDrop = [this](B2SBulb& bulb) { m_physics->DeactivateGravityDrop(bulb); };

   // Startup visibility (ShowStartupImages / ShowStartupSnippits run after the registrations): a picture
   // with InitialState 1 is shown, the others keep the state the registrations gave them
   for (auto* bulbs : { &table.m_backglassIlluminations, &table.m_dmdIlluminations })
      for (auto& bulbPtr : *bulbs)
      {
         if (!bulbPtr->m_pro)
            continue;
         B2SProBulbData& pro = *bulbPtr->m_pro;
         ProBulbRuntime& rt = pro.rt;
         if (pro.initialState == 1)
            rt.visible = true;
         rt.driven = rt.nativeRotation || rt.pivotRotation || rt.motion.active;
         // Runtime driven pictures with a ROM or script id (UsedRomLampIDs / Solenoid / GIString / Switch
         // routes): native rotation pictures turn (SetNativeRotationState), the others are shown or
         // hidden. Pivots have no ROM id (romId is 0).
         if (rt.driven && !rt.triggerIsPivot && pro.romId > 0)
         {
            switch (pro.romIdType)
            {
            case 1:
               rt.triggerRomIdType = static_cast<int>(B2SRomIDType::Lamp);
               rt.triggerFromScript = true; // MyB2SSetData routes lamp-id writes to UsedRomLampIDs
               break;
            case 2: rt.triggerRomIdType = static_cast<int>(B2SRomIDType::Solenoid); break;
            case 3: rt.triggerRomIdType = static_cast<int>(B2SRomIDType::GIString); break;
            case 5: rt.triggerRomIdType = static_cast<int>(B2SRomIDType::Switch); break;
            default: break;
            }
            rt.triggerId = pro.romId;
         }
      }
}

void B2SProRuntime::BindTriggers(const Resolver& resolver)
{
   for (auto* bulbs : { &m_table.m_backglassIlluminations, &m_table.m_dmdIlluminations })
      for (auto& bulb : *bulbs)
      {
         if (!bulb->m_pro)
            continue;
         ProBulbRuntime& rt = bulb->m_pro->rt;
         rt.romUpdater = []() { };
         if (rt.triggerRomIdType != 0 && rt.triggerId > 0)
            rt.romUpdater = resolver(static_cast<B2SRomIDType>(rt.triggerRomIdType), rt.triggerId, &rt.romValue);
      }
   m_motion->BindTriggers(resolver);
   m_physics->BindTriggers(resolver);
}

void B2SProRuntime::ProcessNamedCommands(B2SServer* server)
{
   if (server == nullptr)
      return;
   for (const auto& [key, value] : server->DrainProNamedCommands())
   {
      // TryMoveExternalPivot: a known target consumes the write, only non zero values move
      // TryMoveExternalPositionGrid runs before TryMoveExternalPivot
      if (m_motion->TryMoveGrid(key, value))
         continue;
      const auto it = m_pivotTargets.find(key);
      if (it == m_pivotTargets.end() || value == 0)
         continue;
      ExternalPivotState& state = m_pivotStates[it->second];
      const auto angle = state.angles.find(key);
      if (state.representative == nullptr || angle == state.angles.end())
         continue;
      ProSetPivotRotationTarget(state.representative->m_pro->rt, angle->second, state.duration);
   }
}

void B2SProRuntime::PollTrigger(B2SServer* server, B2SBulb& bulb)
{
   B2SProBulbData& pro = *bulb.m_pro;
   ProBulbRuntime& rt = pro.rt;
   if (rt.triggerRomIdType == 0 && !rt.triggerFromScript)
      return;
   // Visibility routes skip pictures whose group an animation locks; pivot routes do not check locks
   const bool locked = !rt.triggerIsPivot && server != nullptr && !pro.name.empty() && server->IsIlluminationLocked(pro.name);
   const bool isSwitch = rt.triggerRomIdType == static_cast<int>(B2SRomIDType::Switch);
   const auto apply = [&](bool state)
   {
      if (rt.triggerIsPivot)
         ProSetPivotTriggerState(rt, state);
      else if (!locked)
      {
         if (rt.nativeRotation)
            ProSetNativeRotationState(rt, pro, state != pro.romInverted);
         else
            rt.visible = state != pro.romInverted;
      }
   };

   // ROM state: changes only (the reference acts on the controller's change reports); the first
   // observation counts when it is on, and always for switches (their initial state is applied)
   if (rt.triggerRomIdType != 0)
   {
      rt.romUpdater();
      if (rt.romValue >= 0.f)
      {
         const bool on = isSwitch ? rt.romValue > 0.f : rt.romValue >= 0.5f;
         const bool wasKnown = rt.lastRomValue >= 0.f;
         const bool wasOn = wasKnown && (isSwitch ? rt.lastRomValue > 0.f : rt.lastRomValue >= 0.5f);
         rt.lastRomValue = rt.romValue;
         if (wasKnown ? (on != wasOn) : (on || isSwitch))
            apply(on);
      }
   }

   // Script B2SSetData lamp-id writes
   if (rt.triggerFromScript && server != nullptr)
   {
      float value = 0.f;
      uint64_t stamp = 0;
      if (server->GetScriptedLampState(rt.triggerId, value, &stamp) && stamp != rt.lastScriptStamp)
      {
         rt.lastScriptStamp = stamp;
         const int v = static_cast<int>(value);
         if (rt.triggerIsPivot)
            ProSetPivotTriggerState(rt, v != 0);
         else if (!locked)
         {
            if (pro.romIdValue > 0)
               rt.visible = (pro.romIdValue == v);
            else if (rt.nativeRotation)
               ProSetNativeRotationState(rt, pro, (v != 0) != pro.romInverted);
            else
               rt.visible = (v != 0) != pro.romInverted;
         }
      }
   }
}

void B2SProRuntime::Update(B2SServer* server, const vector<std::unique_ptr<B2SBulb>>& bulbs, float elapsedS)
{
   ProcessNamedCommands(server);
   m_motion->PollTriggers(server);
   m_physics->PollLaunchers(server);
   const double elapsedMs = static_cast<double>(elapsedS) * 1000.0;
   for (const auto& bulbPtr : bulbs)
   {
      if (!bulbPtr->m_pro)
         continue;
      B2SBulb& bulb = *bulbPtr;
      B2SProBulbData& pro = *bulb.m_pro;
      ProBulbRuntime& rt = pro.rt;
      if (!rt.started)
      {
         // formBackglass_Shown: startup pivots swing; continuous native rotations start (and are shown)
         rt.started = true;
         if (rt.startupPivot)
            ProSetPivotTriggerState(rt, true);
         if (rt.nativeRotation && rt.autoStartRotation)
         {
            rt.visible = true;
            ProStartNativeRotation(rt);
         }
      }
      if (!rt.driven)
         continue;
      PollTrigger(server, bulb);
      // Illumination group writes (animations, B2SSetData by name) still switch the picture on and off
      if (server != nullptr && !pro.name.empty())
      {
         float value = 0.f;
         uint64_t stamp = 0;
         if (server->GetScriptedGroupState(pro.name, value, stamp) && stamp > rt.lastGroupStamp)
         {
            rt.lastGroupStamp = stamp;
            rt.visible = value != 0.f;
         }
      }
      ProAdvanceMotion(rt, pro, elapsedMs);
   }
   m_motion->Update(bulbs, elapsedS);
   m_physics->Advance(server);
}

void ProStartNativeRotation(ProBulbRuntime& rt)
{
   if (!rt.nativeRotation)
      return;
   rt.commandedOn = true;
   // A renewed ON command cancels any stop requested while the rotation was still running
   rt.slowDownSteps = 0.0;
   rt.runTillEnd = false;
   rt.runToFirstStep = false;
   if (rt.running)
      return;
   rt.steps = std::clamp(rt.steps, 2, 360);
   rt.interval = std::clamp(rt.interval, 10, 500);
   rt.visible = true;
   rt.running = true;
}

void ProStopNativeRotation(ProBulbRuntime& rt, const B2SProBulbData& pro)
{
   if (!rt.nativeRotation)
      return;
   rt.commandedOn = false;
   if (!rt.running)
      return;
   switch (pro.stopBehaviour)
   {
   case ProStopBehaviour::SpinOff: rt.slowDownSteps = 1.0; break;
   case ProStopBehaviour::RunAnimationTillEnd: rt.runTillEnd = true; break;
   case ProStopBehaviour::RunAnimationToFirstStep: rt.runToFirstStep = true; break;
   default: rt.running = false; break;
   }
}

void ProSetNativeRotationState(ProBulbRuntime& rt, const B2SProBulbData& pro, bool enabled)
{
   if (rt.commandedOn == enabled)
   {
      if (enabled && !rt.running)
         ProStartNativeRotation(rt);
      return;
   }
   rt.commandedOn = enabled;
   if (enabled)
      ProStartNativeRotation(rt);
   else
      ProStopNativeRotation(rt, pro);
}

void ProSetPivotRotationTarget(ProBulbRuntime& rt, float targetAngle, int duration)
{
   // A pivot a physics ball collides with moves in the physics steps
   if (rt.physicsPivot)
   {
      B2SProPhysics::SetPivotRotationTarget(rt, targetAngle, duration);
      return;
   }
   rt.physicsAngularVelocity = 0.f;
   rt.startAngle = rt.angle;
   rt.targetAngle = targetAngle;
   rt.tweenDuration = std::clamp(duration, 10, rt.automaticOscillation ? 5000 : 1000);
   rt.pivotRotation = true;
   rt.nativeRotation = true;
   rt.driven = true;
   rt.visible = true;
   rt.tweenElapsedMs = 0.0;
   rt.moving = true;
}

void ProSetPivotTriggerState(ProBulbRuntime& rt, bool enabled)
{
   if (rt.automaticOscillation)
   {
      if (enabled == rt.automaticActive)
         return;
      rt.automaticActive = enabled;
      // Automatic pivots swing between the limits but rest in the unrotated pose
      ProSetPivotRotationTarget(rt, enabled ? rt.upAngle : 0.f, rt.moveDuration);
      return;
   }
   ProSetPivotRotationTarget(rt, enabled ? rt.upAngle : rt.downAngle, rt.moveDuration);
}

void ProAdvanceMotion(ProBulbRuntime& rt, const B2SProBulbData& pro, double elapsedMs)
{
   if (rt.running)
   {
      // NativeRotationTimer_Tick
      const float stepAngle = 360.f / static_cast<float>(std::max(4, rt.steps));
      double speedFactor = 1.0;
      int slowDownTickCount = 0;
      if (rt.slowDownSteps > 0.0)
      {
         slowDownTickCount = 25 + static_cast<int>(std::ceil(900.0 / std::max(1, rt.interval)));
         speedFactor = std::max(0.0, 1.0 - (rt.slowDownSteps - 1.0) / std::max(1, slowDownTickCount));
      }
      const float nextProgress = rt.progress + static_cast<float>(stepAngle * elapsedMs / std::max(1, rt.interval) * speedFactor);
      const bool completedRotation = nextProgress >= 360.f;
      rt.progress = std::fmod(nextProgress, 360.f);
      rt.angle = pro.rotationDirection == 1 ? -rt.progress : rt.progress;
      rt.visible = true;
      if (rt.slowDownSteps > 0.0)
      {
         rt.slowDownSteps += elapsedMs / std::max(1, rt.interval);
         if (rt.slowDownSteps >= slowDownTickCount + 1.0)
         {
            rt.running = false;
            rt.slowDownSteps = 0.0;
         }
      }
      else if (rt.runTillEnd && completedRotation)
      {
         rt.runTillEnd = false;
         rt.running = false;
      }
      else if (rt.runToFirstStep && completedRotation)
      {
         rt.runToFirstStep = false;
         rt.angle = 0.f;
         rt.running = false;
      }
   }

   if (rt.moving)
   {
      // PivotRotationTimer_Tick (smoothstep from the start to the target angle)
      rt.tweenElapsedMs += elapsedMs;
      const float progress = static_cast<float>(std::min(1.0, rt.tweenElapsedMs / rt.tweenDuration));
      const float eased = progress * progress * (3.f - 2.f * progress);
      rt.angle = rt.startAngle + (rt.targetAngle - rt.startAngle) * eased;
      if (progress >= 1.f)
      {
         rt.moving = false;
         rt.angle = rt.targetAngle;
         if (rt.automaticOscillation && rt.automaticActive)
            ProSetPivotRotationTarget(rt, std::abs(rt.targetAngle - rt.upAngle) < 0.01f ? rt.downAngle : rt.upAngle, rt.moveDuration);
      }
   }
}

}
