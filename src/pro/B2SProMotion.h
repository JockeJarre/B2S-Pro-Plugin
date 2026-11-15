// license:GPLv3+

// B2S Pro: motion paths, exit paths, sequence troughs and external position grids, a port of the
// motion members of B2SPictureBox and of B2SData.MotionPathSequenceState / ExternalPositionGridState
// (B2S Pro Server). Timers are replaced by the frame time; events are dispatched synchronously.

#pragma once

#include "common.h"
#include "B2SDataModel.h"
#include "pro/B2SProData.h"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace B2S {

class B2SServer;

class B2SProMotionSystem final
{
public:
   explicit B2SProMotionSystem(B2STable& table);

   using Resolver = std::function<std::function<void()>(B2SRomIDType romIdType, int romId, float* target)>;
   void BindTriggers(const Resolver& resolver);

   // Polls the motion triggers (ROM lamps/solenoids/switches and script B2SSetData ids)
   void PollTriggers(B2SServer* server);
   // Advances the motion of the given window's pictures
   void Update(const vector<std::unique_ptr<B2SBulb>>& bulbs, float elapsedS);
   // B2SSetData(name, value) on a position grid target; true when the name is a grid target
   bool TryMoveGrid(const string& key, int value);

   // Gravity drop hand-off to the physics balls (B2SData.ActivateGravityDrop / DeactivateGravityDrop)
   std::function<bool(B2SBulb&)> activateGravityDrop = [](B2SBulb&) { return false; };
   std::function<void(B2SBulb&)> deactivateGravityDrop = [](B2SBulb&) { };

   // Motion primitives (B2SPictureBox)
   void StartMotionPath(B2SBulb& bulb) { StartCore(bulb, false, false); }
   void StartMotionPathFromCurrentPosition(B2SBulb& bulb) { StartCore(bulb, false, true); }
   void StartMotionPathExit(B2SBulb& bulb) { StartCore(bulb, true, false); }
   void StopMotionPath(B2SBulb& bulb);
   void ResumeMotionPath(B2SBulb& bulb);
   void SetMotionPathPosition(B2SBulb& bulb, ProPoint center, bool advanceRoll = false);
   static ProPoint Center(const B2SBulb& bulb);

   static void InitRect(B2SBulb& bulb);

private:
   B2STable& m_table;

   void StartCore(B2SBulb& bulb, bool useExitPath, bool startFromCurrentPosition);
   void StartShift(B2SBulb& bulb, ProPoint target, int duration, int delayMs);
   void StartExternalGridShift(B2SBulb& bulb, ProPoint target, int duration, ProPoint lookAhead);
   void StartDrainAll(B2SBulb& bulb, const vector<ProPoint>& route, int duration);
   vector<ProPoint> ExitRuntimeRoute(const B2SBulb& bulb) const;
   void StartCollision(B2SBulb& bulb, ProPoint direction, float amplitude, int delayMs);
   void StopCollision(B2SBulb& bulb, bool restorePosition);
   bool StartSourceRespawn(B2SBulb& bulb);
   void StopSourceRespawn(B2SBulb& bulb);
   void SetSourceAnchor(B2SBulb& bulb, bool enabled);
   void SetSourceImage(B2SBulb& bulb, VPXTexture image);
   void SetSourceVisible(B2SBulb& bulb, bool visible);
   void SetCenter(B2SBulb& bulb, ProPoint center, bool advanceRoll);
   void AdvanceRoll(const B2SBulb& bulb, ProPoint previous, ProPoint current, float visualW, float visualH, float& angle) const;
   void Tick(B2SBulb& bulb, double elapsedMs);

   // Events (MotionPathCompleted, MotionPathLaunchSegmentCompleted, MotionPathSourceRespawnCompleted)
   void OnCompleted(B2SBulb& bulb);
   void OnLaunchSegmentCompleted(B2SBulb& bulb);
   void OnSourceRespawnCompleted(B2SBulb& bulb);

   // B2SData.MotionPathSequenceState
   struct Sequence
   {
      vector<B2SBulb*> members;
      vector<B2SBulb*> activeEntries;
      vector<B2SBulb*> occupied;
      vector<bool> pendingOperations; // Queue of remove flags
      vector<ProPoint> compactionTargets;
      vector<B2SBulb*> activeDrainAllRemovals;
      vector<B2SBulb*> availableGravityMembers;
      B2SBulb* activeRemoval = nullptr;
      B2SBulb* launchingMember = nullptr;
      B2SBulb* sourceAnchor = nullptr;
      bool sourceReady = true;
      bool gravityReuseEnabled = false;
   };
   vector<Sequence> m_sequences;
   std::unordered_map<string, size_t> m_sequenceByName;
   void SequenceRegister(Sequence& s, B2SBulb* bulb);
   void SequenceRequestStart(Sequence& s);
   void SequenceRequestRemove(Sequence& s);
   void SequenceQueue(Sequence& s, bool remove);
   bool SequenceQueueEnabled(const Sequence& s) const;
   bool SequenceDrainAllEnabled(const Sequence& s) const;
   bool SequenceGravityDropEnabled(const Sequence& s) const;
   bool SequenceStartEntry(Sequence& s);
   void SequenceAssignNextOpenEntrySlot(Sequence& s, B2SBulb* captured);
   bool SequenceStartRemoval(Sequence& s);
   bool SequenceStartAllRemovals(Sequence& s);
   bool SequenceStartGravityDrop(Sequence& s);
   void SequenceBeginEntry(Sequence& s, B2SBulb* member, bool startFromCurrentPosition);
   void SequenceBeginSourceRespawn(Sequence& s);
   void SequenceCompleteSourceRespawn(Sequence& s);
   void SequenceProcessPending(Sequence& s);
   void SequenceStartCompactionShifts(Sequence& s, B2SBulb* removing);
   void SequenceNormalizeCompactedSlots(Sequence& s, int remainingCount);
   void SequenceUpdateSourceAppearance(Sequence& s);
   void SequenceEntryCompleted(Sequence& s, B2SBulb* member);
   void SequenceApplyEntryCollision(Sequence& s, B2SBulb* incoming);
   void SequenceRemovalCompleted(Sequence& s, B2SBulb* member);
   void SequenceDrainAllRemovalCompleted(Sequence& s, B2SBulb* member);
   void StartMotionPaths(const vector<B2SBulb*>& bulbs);
   void RemoveMotionPaths(const vector<B2SBulb*>& bulbs);

   // B2SData.ExternalPositionGridState
   struct Grid
   {
      B2SBulb* representative = nullptr;
      int duration = 45;
      std::unordered_map<string, B2SBulb*> targets;
      string pending;
      bool flushOn = false;
      double flushRemainingMs = 0.0;
   };
   vector<Grid> m_grids;
   std::unordered_map<string, size_t> m_gridTargets;
   std::unordered_map<string, size_t> m_gridByName;

   // Trigger routes (B2SData.UsedMotionPath*IDs) keyed by (source type, id)
   struct Route
   {
      vector<B2SBulb*> start, stop, resume, remove;
   };
   std::map<std::pair<int, int>, Route> m_routes;
   struct EdgeSource
   {
      float value = -1.f;
      float last = -1.f;
      std::function<void()> updater = []() { };
   };
   std::map<std::pair<int, int>, EdgeSource> m_edges; // ROM sources (lamp 2, solenoid 1, switch 5)
   std::map<int, std::pair<uint64_t, int>> m_b2sStates; // Script ids: last stamp and value (motionPathB2SStates)
   std::unordered_map<B2SBulb*, bool> m_startupPending; // Motion paths without a start trigger (ShowStartupImages starts them)
   void FireRoute(int type, int id, bool scriptOrder);

public:
   // Persistent sequence feeder of a picture (DrawPersistentMotionPathSource)
   static bool IsSourceSeparate(const B2SBulb& bulb);
   static ProRect SourceRect(const B2SBulb& bulb);
   static VPXTexture SourceImage(const B2SBulb& bulb);
   static float SourceRollAngle(const B2SBulb& bulb);
};

}
