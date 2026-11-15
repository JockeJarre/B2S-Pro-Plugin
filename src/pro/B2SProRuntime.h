// license:GPLv3+

// B2S Pro: runtime of the B2S Pro picture behaviours (native one-image rotation, pivots, external pivots,
// named Down/Up pivot commands), a port of B2SPictureBox and B2SData of the B2S Pro Server.
// Only used for B2S Pro files (Illumination/@UnifiedLayerDeck = 1).

#pragma once

#include "common.h"
#include "B2SDataModel.h"
#include "pro/B2SProData.h"
#include "pro/B2SProMotion.h"
#include "pro/B2SProPhysics.h"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace B2S {

class B2SServer;

// Names that B2SSetData(name, value) routes to a B2S Pro movement instead of an illumination group
// (B2SData.TryMoveExternalPositionGrid / TryMoveExternalPivot), lower case and trimmed
vector<string> ProNamedCommandTargets(const B2STable& table);
string ProCommandKey(const string& name);

class B2SProRuntime final
{
public:
   explicit B2SProRuntime(B2STable& table);

   // Rebinds the ROM state readers of the triggers (called when the controller state sources change)
   using Resolver = std::function<std::function<void()>(B2SRomIDType romIdType, int romId, float* target)>;
   void BindTriggers(const Resolver& resolver);

   // Per frame, before the bulbs of the given window are updated
   void Update(B2SServer* server, const vector<std::unique_ptr<B2SBulb>>& bulbs, float elapsedS);

private:
   B2STable& m_table;

   struct ExternalPivotState
   {
      B2SBulb* representative = nullptr;
      int duration = 80;
      std::unordered_map<string, float> angles; // Target name (lower case) -> angle
   };
   vector<ExternalPivotState> m_pivotStates;
   std::unordered_map<string, size_t> m_pivotTargets; // Target name (lower case) -> state index
   std::unique_ptr<B2SProMotionSystem> m_motion;
   std::unique_ptr<B2SProPhysics> m_physics;

public:
   B2SProMotionSystem& Motion() { return *m_motion; }

private:

   void ProcessNamedCommands(B2SServer* server);
   static void PollTrigger(B2SServer* server, B2SBulb& bulb);
};

// Native rotation and pivot motion (B2SPictureBox port)
void ProStartNativeRotation(ProBulbRuntime& rt);
void ProStopNativeRotation(ProBulbRuntime& rt, const B2SProBulbData& pro);
void ProSetNativeRotationState(ProBulbRuntime& rt, const B2SProBulbData& pro, bool enabled);
void ProSetPivotRotationTarget(ProBulbRuntime& rt, float targetAngle, int duration);
void ProSetPivotTriggerState(ProBulbRuntime& rt, bool enabled);
void ProAdvanceMotion(ProBulbRuntime& rt, const B2SProBulbData& pro, double elapsedMs);

}
