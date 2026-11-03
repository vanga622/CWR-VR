#include <Poseidon/World/Entities/Infantry/SoldierOldCommon.hpp>
#include <Poseidon/Core/Application.hpp>
#include <Poseidon/Input/InputSubsystem.hpp>
#include <algorithm>
#include <limits.h>
#include <utility>
#include <Poseidon/Foundation/Common/FltOpts.hpp>
#include <Poseidon/Foundation/Containers/Array.hpp>
#include <Poseidon/Foundation/Containers/BankArray.hpp>
#include <Poseidon/Foundation/Containers/BoolArray.hpp>
#include <Poseidon/Foundation/Containers/StaticArray.hpp>
#include <Poseidon/Foundation/Framework/DebugLog.hpp>
#include <Poseidon/Foundation/Framework/Log.hpp>
#include <Poseidon/Foundation/Math/Math3D.hpp>
#include <Poseidon/Foundation/Math/Math3DP.hpp>
#include <Poseidon/Foundation/Math/MathDefs.hpp>
#include <Poseidon/Foundation/Memory/FastAlloc.hpp>
#include <Poseidon/Foundation/Strings/RString.hpp>
#include <Poseidon/Foundation/Time/Time.hpp>
#include <Poseidon/Foundation/Types/LLinks.hpp>
#include <Poseidon/Foundation/Types/Pointers.hpp>

#pragma warning(disable : 4065)

using namespace Poseidon;
ActionContextBase* CreateGetInActionContext(Transport* veh, UIActionType pos)
{
    using namespace Poseidon;
    return new ActionContextGetIn(veh, pos);
}
namespace Poseidon
{

DEFINE_FAST_ALLOCATOR(ActionContextDefault)

ActionContextDefault::ActionContextDefault()
{
    static const RStringB empty = "";
    param = 0;
    param2 = 0;
    param3 = empty;
}

ActionContextDefault::~ActionContextDefault() = default;

DEFINE_FAST_ALLOCATOR(ActionContextUIAction)

ActionContextUIAction::ActionContextUIAction(const UIAction& uiAction) : action(uiAction)
{
    function = MFUIAction;
}

DEFINE_FAST_ALLOCATOR(ActionContextGetIn)

ActionContextGetIn::ActionContextGetIn(Transport* veh, UIActionType pos)
{
    vehicle = veh;
    position = pos;
}

const static MotionEdge NoEdge;

MotionPath::MotionPath() {}

BankArray<AnimationRT> AnimationRTBank;
BankArray<WeightInfo> WeigthBank;

MotionPath::~MotionPath() = default;

MotionType::MotionType()
{
    _entry = nullptr;
    _skeleton = new Skeleton("");
}
MotionType::~MotionType() = default;

void MotionType::AssignSkeleton(RStringB name)
{
    _skeleton = new Skeleton(name);
}

RStringB MotionType::GetMoveName(MoveId id) const
{
    if (id == MoveIdNone)
    {
        return RStringB("<none>");
    }
    return _moveIds.GetName(id);
}

MoveId MotionType::GetMoveId(RStringB name) const
{
    int value = _moveIds.GetValue(name);
    if (value < 0 && name.GetLength() > 0)
    {
        LOG_DEBUG(Physics, "Warning: unknown value '{}'", (const char*)name);
    }
    return MoveId(value);
}

struct UsedContext
{
    AutoArray<bool> _indexUsed;
};

const MotionEdge& MotionType::Edge(MoveId a, MoveId b) const
{
    // find edge in edge list
    PoseidonAssert(a >= 0);
    PoseidonAssert(b >= 0);
    const MotionEdges& edges = _vertex[a];
    for (int i = 0; i < edges.Size(); i++)
    {
        if (edges[i].target == b)
        {
            return edges[i];
        }
    }
    return NoEdge;
}

int MotionType::EdgeCost(MoveId a, MoveId b) const
{
    const MotionEdge& edge = Edge(a, b);
    if (edge.cost < 0)
    {
        return INT_MAX;
    }
    return edge.cost;
}

void MotionType::AddEdge(MoveId a, MoveId b, MotionEdgeType type, float cost)
{
    int iCost = toInt(cost * 50);
    MotionEdges& edges = _vertex[a];
    for (int i = 0; i < edges.Size(); i++)
    {
        MotionEdge& edge = edges[i];
        if (edge.target == b)
        {
            edge.type = type;
            edge.cost = iCost;
            return;
        }
    }
    edges.Add(MotionEdge(b, iCost, type));
}

void MotionType::DeleteEdge(MoveId a, MoveId b)
{
    MotionEdges& edges = _vertex[a];
    for (int i = 0; i < edges.Size(); i++)
    {
        MotionEdge& edge = edges[i];
        if (edge.target == b)
        {
            edges.Delete(i);
            return;
        }
    }
}

#define LOG_PATH 0

bool MotionType::FindPath(MotionPath& path, MoveId from, MotionPathItem to) const
{
    path.Resize(0);
    int nVerts = _vertex.Size();
    if (from < 0 || from >= nVerts || to.id < 0 || to.id >= nVerts)
    {
        return false;
    }

    const MotionEdge& edge = Edge(from, to.id);
    if ((MotionEdgeType)edge.type != MEdgeNone)
    {
        path.Add(to);
#if LOG_PATH
        LOG_DEBUG(Physics, "Direct path from {} to {}", NAME(from), NAME(to.id));
        LOG_DEBUG(Physics, "  insert {}, cost {}", NAME(to.id), edge.cost);
#endif
        return true;
    }

    AUTO_STATIC_ARRAY(int, vs, 1024);
    AUTO_STATIC_ARRAY(int, d, 1024);
    AUTO_STATIC_ARRAY(int, p, 1024);
    d.Resize(nVerts);
    p.Resize(nVerts);
    for (int i = 0; i < nVerts; i++)
    {
        d[i] = INT_MAX, p[i] = -1;
    }
    for (int i = 0; i < nVerts; i++)
    {
        vs.Add(i);
    }
    d[from] = 0;
    const int unaccessible = INT_MAX / 16;

    for (;;)
    {
        // extract cheapest: may be faster using heap sort, but we do not care
        int minI = -1, minVal = unaccessible;
        for (int i = 0; i < vs.Size(); i++)
        {
            int index = vs[i];
            int val = d[index];
            if (minVal > val)
            {
                minVal = val, minI = i;
            }
        }
        if (minI < 0)
        {
            break; // nothing to add
        }
        int v = vs[minI];
        vs.Delete(minI);
        int vCost = d[v];
        const MotionEdges& e = _vertex[v];
        for (int i = 0; i < e.Size(); i++)
        {
            const MotionEdge& ei = e[i];
            int target = ei.target;
            if (target < 0 || target >= nVerts)
            {
                continue;
            }
            int costToTarget = vCost + ei.cost;
            if (costToTarget < d[target])
            {
                d[target] = costToTarget, p[target] = v;
            }
        }
    }

    if (p[to.id] < 0)
    {
        return false;
    }

    path.Add(to);
#if LOG_PATH
    LOG_DEBUG(Physics, "State search from {} to {}", NAME(from), NAME(to.id));
    LOG_DEBUG(Physics, "  insert {}, cost {}", NAME(to.id), d[to.id]);
#endif
    for (;;)
    {
        int predecessor = p[to.id];
        if (predecessor < 0 || predecessor >= nVerts)
        {
            path.Resize(0);
            return false;
        }
        to = MotionPathItem((MoveId)predecessor);
        if (to.id == from)
        {
#if LOG_PATH
            LOG_DEBUG(Physics, "  found {}, cost {}", NAME(from), d[from]);
#endif
            return true;
        }
        path.Insert(0, to);
#if LOG_PATH
        LOG_DEBUG(Physics, "  insert {}, cost {}", NAME(to.id), d[to.id]);
#endif
    }
}

ActionVehMap::ActionVehMap() = default;

void ActionVehMap::Load(const MotionType* motion, const ParamEntry& map)
{
    RString prefix = "ManAct";
    int nAct = GActionVehNames.FirstInvalidValue();
    _actionMoves.Realloc(nAct);
    _actionMoves.Resize(nAct);
    for (int id = 0; id < nAct; id++)
    {
        _actionMoves[id] = MoveIdNone;
    }

    for (int i = 0; i < map.GetEntryCount(); i++)
    {
        const ParamEntry& entry = map.GetEntry(i);
        RStringB name = entry.GetName();
        RStringB value = entry;
        RString fullName = prefix + name;
        const char* str = fullName;
        int idInt = GActionVehNames.GetValue(str);
        if (idInt < 0)
        {
            RptF("Bad action %s in %s", (const char*)str, (const char*)map.GetName());
            continue;
        }
        ManVehAction id = (ManVehAction)idInt;
        PoseidonAssert(_actionMoves.Size() > id);
        _actionMoves[id] = motion->GetMoveId(value);
        if (_actionMoves[id] == MoveIdNone)
        {
            RptF("Invalid move %s in action %s (%s)", (const char*)value, (const char*)name,
                 (const char*)map.GetContext());
        }
    }
}

} // namespace Poseidon
void InitMan()
{
    using namespace Poseidon;
    const ParamEntry& map = Pars >> "CfgVehicleActions";
    RString prefix = "ManAct";
    for (int i = 0; i < map.GetEntryCount(); i++)
    {
        const ParamEntry& entry = map.GetEntry(i);
        RStringB name = entry.GetName();
        GActionVehNames.AddValue(prefix + name);
    }
}
namespace Poseidon
{

ActionMap* MotionType::NewActionMap(const ParamEntry* cfg)
{
    ActionMapName name;
    name.motion = this;
    name.entry = cfg;
    return _actionMaps.New(name);
}

BlendAnimType* MotionType::NewBlendAnimType(const ParamEntry& cfg)
{
    BlendAnimTypeName name;
    name.motion = this;
    name.cfg = &cfg;
    return _blendAnimTypes.New(name);
}

void MotionType::InitNoActions(const ParamEntry* cfg)
{
    _noActions = NewActionMap(cfg);
}

MoveId MotionType::GetDefaultMove(int upDegree) const
{
    for (int i = 0; i < _actionMaps.Size(); i++)
    {
        const ActionMap* map = _actionMaps[i];
        if (map->GetUpDegree() == upDegree)
        {
            return map->GetAction(ManActDefault);
        }
    }
    return MoveIdNone;
}

MoveId MotionType::GetMove(int upDegree, ManAction action) const
{
    for (int i = 0; i < _actionMaps.Size(); i++)
    {
        const ActionMap* map = _actionMaps[i];
        if (map->GetUpDegree() == upDegree)
        {
            return map->GetAction(action);
        }
    }
    return MoveIdNone;
}

static void CheckArrayMod(const ParamEntry& entry, int mod, const ParamEntry* context = nullptr)
{
    if (entry.GetSize() % mod != 0)
    {
        RptF("%s: item count not multiple of %d (is %d)",
             context ? (const char*)context->GetContext(entry.GetName()) : (const char*)entry.GetName(), mod,
             entry.GetSize());
    }
}

static void BadMove(const ParamEntry& entry, const char* name, const ParamEntry* context = nullptr)
{
    RptF("  %s: Bad move %s",
         context ? (const char*)context->GetContext(entry.GetName()) : (const char*)entry.GetName(), (const char*)name);
}

void MotionType::Load(const ParamEntry& entry)
{
    _skeleton = Skeletons.New(entry.GetName());
    _entry = &entry;
    const ParamEntry& states = entry >> "States";

    _moveIds.Clear();
    for (int i = 0; i < states.GetEntryCount(); i++)
    {
        const ParamEntry& sub = states.GetEntry(i);
        _moveIds.AddValue(sub.GetName());
    }
    _moveIds.Close();

    int moveIdN = MoveIdN();
    _vertex.Realloc(moveIdN);
    _vertex.Resize(moveIdN);

    // load interpolations information
    const ParamEntry& inter = entry >> "Interpolations";
    for (int i = 0; i < inter.GetEntryCount(); i++)
    {
        const ParamEntry& group = inter.GetEntry(i);
        float fCost = group[0];
        // insert all possible pairs of states
        for (int s = 1; s < group.GetSize(); s++)
        {
            RStringB sName = group[s];
            MoveId sMove = GetMoveId(sName);
            if (sMove == MoveIdNone)
            {
                continue;
            }

            for (int r = 1; r < group.GetSize(); r++)
            {
                if (r != s)
                {
                    RStringB rName = group[r];
                    MoveId rMove = GetMoveId(rName);
                    if (rMove == MoveIdNone)
                    {
                        continue;
                    }

                    AddEdge(rMove, sMove, MEdgeInterpol, fCost);
                }
            }
        }
    }

    { // interpolated transitions
        const ParamEntry& trans = entry >> "transitionsInterpolated";
        CheckArrayMod(trans, 3, &entry);
        for (int i = 0; i < trans.GetSize(); i += 3)
        {
            RStringB a = trans[i];
            RStringB b = trans[i + 1];
            float fCost = trans[i + 2];
            MoveId aId = GetMoveId(a);
            MoveId bId = GetMoveId(b);
            if (aId == MoveIdNone || bId == MoveIdNone)
            {
                RptF("Bad ipol transition from %s to %s", (const char*)a, (const char*)b);
                continue;
            }
            AddEdge(aId, bId, MEdgeInterpol, fCost);
        }
    }
    { // simple transitions
        const ParamEntry& trans = entry >> "transitionsSimple";

        CheckArrayMod(trans, 3, &entry);
        for (int i = 0; i < trans.GetSize(); i += 3)
        {
            RStringB a = trans[i];
            RStringB b = trans[i + 1];
            float fCost = trans[i + 2];
            MoveId aId = GetMoveId(a);
            MoveId bId = GetMoveId(b);
            if (aId == MoveIdNone || bId == MoveIdNone)
            {
                RptF("Bad simple transition from %s to %s", (const char*)a, (const char*)b);
                continue;
            }
            AddEdge(aId, bId, MEdgeSimple, fCost);
        }
    }

    for (int i = 0; i < moveIdN; i++)
    {
        const ParamEntry& state = states.GetEntry(i);
        const ParamEntry& conFrom = state >> "connectFrom";
        CheckArrayMod(conFrom, 2, &state);
        for (int s = 0; s < conFrom.GetSize() - 1; s += 2)
        {
            RStringB name = conFrom[s];
            float cost = conFrom[s + 1];
            MoveId id = GetMoveId(name);
            if (id == MoveIdNone)
            {
                BadMove(conFrom, name, &state);
            }
            else
            {
                AddEdge(id, (MoveId)i, MEdgeSimple, cost);
            }
        }
        const ParamEntry& conTo = state >> "connectTo";
        CheckArrayMod(conTo, 2, &state);
        for (int s = 0; s < conTo.GetSize() - 1; s += 2)
        {
            RStringB name = conTo[s];
            float cost = conTo[s + 1];
            MoveId id = GetMoveId(name);
            if (id == MoveIdNone)
            {
                BadMove(conTo, name, &state);
            }
            else
            {
                AddEdge((MoveId)i, id, MEdgeSimple, cost);
            }
        }

        const ParamEntry& ipolW = state >> "interpolateWith";
        CheckArrayMod(ipolW, 2, &state);
        for (int s = 0; s < ipolW.GetSize() - 1; s += 2)
        {
            RStringB name = ipolW[s];
            float cost = ipolW[s + 1];
            MoveId id = GetMoveId(name);
            if (id == MoveIdNone)
            {
                BadMove(ipolW, name, &state);
            }
            else
            {
                AddEdge((MoveId)i, id, MEdgeInterpol, cost);
                AddEdge(id, (MoveId)i, MEdgeInterpol, cost);
            }
        }

        const ParamEntry& ipolT = state >> "interpolateTo";
        CheckArrayMod(ipolT, 2, &state);
        for (int s = 0; s < ipolT.GetSize() - 1; s += 2)
        {
            RStringB name = ipolT[s];
            float cost = ipolT[s + 1];
            MoveId id = GetMoveId(name);
            if (id == MoveIdNone)
            {
                BadMove(ipolT, name, &state);
            }
            else
            {
                AddEdge((MoveId)i, id, MEdgeInterpol, cost);
            }
        }

        const ParamEntry& ipolF = state >> "interpolateFrom";
        CheckArrayMod(ipolF, 2, &state);
        for (int s = 0; s < ipolF.GetSize() - 1; s += 2)
        {
            RStringB name = ipolF[s];
            float cost = ipolF[s + 1];
            MoveId id = GetMoveId(name);
            if (id == MoveIdNone)
            {
                BadMove(ipolF, name, &state);
            }
            else
            {
                AddEdge(id, (MoveId)i, MEdgeInterpol, cost);
            }
        }
    }

    for (int i = 0; i < moveIdN; i++)
    {
        const ParamEntry& state = states.GetEntry(i);
        const RStringB& asState = state >> "connectAs";
        if (asState.GetLength() <= 0)
        {
            continue;
        }
        MoveId as = GetMoveId(asState);
        if (as == MoveIdNone)
        {
            continue;
        }
        for (int j = 0; j < moveIdN; j++)
        {
            const MotionEdge& sEdge = Edge(as, (MoveId)j);
            if ((MotionEdgeType)sEdge.type == MEdgeNone)
            {
                continue;
            }
            const MotionEdge& tEdge = Edge((MoveId)i, (MoveId)j);
            if ((MotionEdgeType)tEdge.type != MEdgeNone)
            {
                continue;
            }
            AddEdge((MoveId)i, (MoveId)j, sEdge.type, sEdge.GetCost());
        }
        for (int j = 0; j < moveIdN; j++)
        {
            const MotionEdge& sEdge = Edge((MoveId)j, as);
            if ((MotionEdgeType)sEdge.type == MEdgeNone)
            {
                continue;
            }
            const MotionEdge& tEdge = Edge((MoveId)j, (MoveId)i);
            if ((MotionEdgeType)tEdge.type != MEdgeNone)
            {
                continue;
            }
            AddEdge((MoveId)j, (MoveId)i, sEdge.type, sEdge.GetCost());
        }
    }

    { // disabled transitions
        const ParamEntry& trans = entry >> "transitionsDisabled";
        CheckArrayMod(trans, 2, &entry);

        for (int i = 0; i < trans.GetSize(); i += 2)
        {
            RStringB a = trans[i];
            RStringB b = trans[i + 1];
            MoveId aId = GetMoveId(a);
            MoveId bId = GetMoveId(b);
            if (aId == MoveIdNone || bId == MoveIdNone)
            {
                RptF("Bad disabled transition from %s to %s", (const char*)a, (const char*)b);
                continue;
            }
            DeleteEdge(aId, bId);
        }
    }

    _vertex.Compact();
    for (int i = 0; i < _vertex.Size(); i++)
    {
        _vertex[i].Compact();
    }

    RStringB vehicleActionsName = entry >> "vehicleActions";
    _actionVehMap.Load(this, Pars >> vehicleActionsName);
}

void MotionType::Unload()
{
    _moveIds.Clear();
    _vertex.Clear();
    _entry = nullptr;
}

const float TimeToRun = 6;
const float TimeToCrawl = 8;
const float WaitBeforeStandUp = TimeToRun + TimeToCrawl;

int Man::GetActUpDegree() const
{
    ActionMap* map = Type()->GetActionMap(_primaryMove.id);
    return map ? map->GetUpDegree() : ManPosStand;
}

DEFINE_CASTING(Man)

Man::Man(VehicleType* name, bool fullCreate)
    : Person(name, fullCreate),

      _turnToDo(0), _walkToggle(false), _inBuilding(false),

      _doSoundStep(false), _freeFallUntil(Glob.time - 60), _stillMoveQueueEnd(MoveIdNone), _variantTime(Glob.time),

      _aimInaccuracyX(0), _aimInaccuracyY(0), _aimInaccuracyDist(0), _lastInaccuracyTime(Glob.time),
      _lastInaccuracyDistTime(Glob.time),

      _lastObjectContactTime(Glob.time), _lastMovementTime(Glob.time),

      _waterDepth(0), _unitPos(UPAuto),

      _hideBody(0), _hideBodyWanted(0),

      _primaryMove((MoveId)MoveIdNone), _secondaryMove((MoveId)MoveIdNone), _externalMove((MoveId)MoveIdNone),
      _externalMoveFinished(false), _forceMove((MoveId)MoveIdNone), _hasNVG(false),

      _upDegreeChangeTime(Glob.time - 5), _upDegreeStable(ManPosStand),

      _posWanted(ManPosStand), _posWantedTime(TIME_MAX),

      _primaryFactor(1), _primaryTime(0), _secondaryTime(0),

      _walkSpeedWanted(0),

      _aimingPositionWorld(VZero), _cameraPositionWorld(VZero),

      _whenKilled(0), _whenScreamed(TIME_MAX),

      _tired(0), _canMoveFast(true),

      _showPrimaryWeapon(true), _showSecondaryWeapon(true), _showHead(true), _manScale(1),

      _nvg(false),

      _gunTrans(MIdentity), _headTrans(MIdentity), _headTransIdent(true), _gunTransIdent(true),

      _correctBankSin(0), _correctBankCos(1), _legTrans(MIdentity),

      _gunYRot(0), _gunYRotWanted(0), _gunXRot(0), _gunXRotWanted(0), _gunXSpeed(0), _gunYSpeed(0),

      _headYRot(0), _headYRotWanted(0), _headXRot(0), _headXRotWanted(0),

      _lookForwardTimeLeft(0), _lookTargetTimeLeft(0),

      _ladderBuilding(nullptr), _ladderIndex(-1), _ladderPosition(0),

      _handGun(false),

      _turnWanted(0), _head(Type()->_head, _shape)
{
    SetSimulationPrecision(1.0f / 5); // 5 times per sec is enough
    RecalcGunTransform();

    _destrType = DestructMan;

    _mGunClouds.Load((*Type()->_par) >> "MGunClouds");
    _gunClouds.Load((*Type()->_par) >> "GunClouds");
    _mGunFireFrames = 0;
    _mGunFireTime = UITIME_MIN;
    _mGunFirePhase = 0;

    _head.SetMimicMode("neutral");
    ScanNVG();
}

Man::~Man() {}

int Man::InsideLOD(CameraType camType) const
{
    return Type()->_insideView;
}

void Man::BlendMatrix(Matrix4& mat, const Matrix4& trans, float factor) const
{
    if (factor >= 0.99f)
    {
        mat = trans * mat;
    }
    else if (factor >= 0.01f)
    {
        // interpolation necessary
        mat = (trans * factor + MIdentity * (1 - factor)) * mat;
    }
}

void Man::AnimateMatrix(Matrix4& mat, const AnimationRTWeight& wgt) const
{
    const ManType* type = Type();
    Matrix4 res = MZero;
    if (_primaryFactor > 0.01f)
    {
        res = mat;
        AnimationRT* anim = type->GetAnimation(_primaryMove.id);
        if (anim)
        {
            anim->Matrix(res, _primaryTime, wgt);
        }
        res = res * _primaryFactor;
    }
    if (_primaryFactor < 0.99f)
    {
        AnimationRT* anim = type->GetAnimation(_secondaryMove.id);
        Matrix4 temp = mat;
        if (anim)
        {
            anim->Matrix(temp, _secondaryTime, wgt);
        }
        res += temp * (1 - _primaryFactor);
    }
    res.Orthogonalize();
    mat = res;
}

const AnimationRTWeight& Man::GetSelWeights(int level, int selection) const
{
    const ManType* type = Type();
    const WeightInfo& weights = type->GetWeights();
    const AnimationRTWeights& wgt = weights[level];

    // check which matrix
    Shape* sShape = _shape->Level(level);
    const NamedSelection& sel = sShape->NamedSel(selection);
    int point = sel[0];
    PoseidonAssert(sel.Size() > 0);
    return wgt[point];
}

const AnimationRTWeight& Man::GetProxyWeights(int level, const ProxyObject& proxy) const
{
    return GetSelWeights(level, proxy.selection);
}

// VR: see SoldierOld.hpp.  The engine's weapon pose is in world space with
// +Z along the barrel; proxy matrices are in this soldier's model space, and
// the barrel runs along the muzzle direction of the weapon's own model (not
// necessarily +Z), so the proxy frame is turned to put that axis on the
// controller's aim.  Proxy +Y is taken as the weapon's up.
Matrix4 Man::AnimateProxyMatrix(int level, const ProxyObject& proxy) const
{
    // The memory level is where the muzzle point and fire direction are
    // read, so it carries the zeroing; drawn levels show the weapon as held.
    Matrix4 vrWeapon;
    if (IsVRHeldWeaponProxy(level, proxy) && VRWeaponModelMatrix(vrWeapon, level == _shape->FindMemoryLevel()))
        return vrWeapon;
    return base::AnimateProxyMatrix(level, proxy);
}

bool Man::VRWeaponModelMatrix(Matrix4& out, bool firing) const
{
    if (_currentWeapon < 0 || !GEngine->VRWeaponPose(out, firing))
        return false;
    Vector3 barrel = GetMagazineSlot(_currentWeapon)._muzzle->_muzzleDir;
    if (barrel.SquareSize() < 1e-6f)
        barrel = VForward;
    Matrix3 proxyAxes;
    proxyAxes.SetDirectionAndUp(barrel, VUp);
    out.SetOrientation(out.Orientation() * proxyAxes.InverseRotation());
    out = GetInvTransform() * out;
    return true;
}

// VR (M5): rigid arm follow.  The animation already holds the weapon with
// both hands; moving the arm bones by the same delta that moves the weapon
// from its animated pose to the controller keeps the hands on it.  Hands and
// forearms move fully, upper arms half way so the elbow bends instead of the
// arm leaving the shoulder.  The left arm rides with the weapon in a
// two-handed hold; otherwise it is moved on from the fore-end (where the
// animation holds it) to the left controller.
bool Man::VRArmsTransform(Matrix4& right, Matrix4& left) const
{
    if (!GWorld || GWorld->CameraOn() != this || GWorld->GetCameraType() != CamInternal || _currentWeapon < 0 ||
        IsDead())
        return false;
    const WeaponType* weapon = GetMagazineSlot(_currentWeapon)._weapon;
    if (!weapon)
        return false;
    const ManType* type = Type();
    int weaponIndex = -1;
    if (weapon->_weaponType & MaskSlotPrimary)
        weaponIndex = type->_priWeaponIndex;
    else if (weapon->_weaponType & MaskSlotSecondary)
        weaponIndex = type->_secWeaponIndex;
    else if (weapon->_weaponType & MaskSlotHandGun)
        weaponIndex = type->_handGunIndex;
    Matrix4 vrWeapon;
    if (weaponIndex < 0 || !VRWeaponModelMatrix(vrWeapon))
        return false;

    const int memory = _shape->FindMemoryLevel();
    const Matrix4 animated = base::AnimateProxyMatrix(memory, _shape->MemoryLevel()->Proxy(weaponIndex));
    right = vrWeapon * Matrix4(MInverseGeneral, animated);
    left = right;

    // Free left hand: from the fore-end, turned by how far the controller is
    // turned from the weapon, and carried to the controller.
    Matrix4 weaponWorld, leftHand;
    if (!GEngine->VRWeaponTwoHanded() && GEngine->VRWeaponPose(weaponWorld) && GEngine->VRHandPose(0, leftHand))
    {
        const Matrix4 toModel = GetInvTransform();
        const Vector3 foreEnd =
            toModel.FastTransform(weaponWorld.Position() + weaponWorld.Direction() * GEngine->VRForeEndForward());
        const Matrix3 turn = (toModel.Orientation() * leftHand.Orientation()) *
                             (toModel.Orientation() * weaponWorld.Orientation()).InverseRotation();
        Matrix4 rotate = MIdentity;
        rotate.SetOrientation(turn);
        left = Matrix4(MTranslation, toModel.FastTransform(leftHand.Position())) * rotate *
               Matrix4(MTranslation, -foreEnd) * right;
    }
    return true;
}

// VR (M5): lying down for real makes the soldier go prone, getting up makes
// him stand.  With a rifle the game has only these two stances
// (ManActDown from standing goes straight to prone, ManActCrouch does
// nothing), so a real crouch only lowers the view (EngineGL33_VR) while the
// soldier keeps standing.  The head drop is measured from the highest head
// position since recentre, i.e. the player's own standing height; 10 cm of
// hysteresis keeps it from flickering.  Rifle only (handgun and launcher
// keep their own transitions).
int Man::VRWantedStanceAction(int actPos) const
{
    float drop = 0.0f;
    if (!GEngine->VRHeadDrop(drop) || IsHandGunSelected() || LauncherSelected() || FindWeaponType(MaskSlotPrimary) < 0)
        return -1;
    const bool prone = IsDown();
    GEngine->VRSetProne(prone);
    static bool s_lastProne = false; // stance changes in the game log
    if (prone != s_lastProne)
    {
        LOG_INFO(Graphics, "VR: soldier {} (head {:.2f} m below standing)", prone ? "prone" : "standing", drop);
        s_lastProne = prone;
    }
    const float proneAt = GEngine->VRProneDrop();
    const float hysteresis = 0.10f;
    if (!prone && drop > proneAt)
        return ManActDown;
    if (prone && drop < proneAt - hysteresis)
        return ManActUp;
    return -1;
}

// VR (M6): how each magazine type feeds its weapon, and which model shows
// the magazine.  The game data has only two hand-sized magazine models that
// no config uses: misc\mag_nato.p3d (STANAG, M16/M4 texture) and
// misc\mag_ak.p3d (AK).  Other detachable magazines get mag_nato as an
// explicitly logged placeholder.  Anything not listed (belts, cylinders,
// internal magazines, grenade and rocket launchers) is not reloaded by hand.
namespace
{
struct VRMagazineFeed
{
    const char* magazine; // CfgWeapons magazine class
    const char* model;    // model in the hand and on the ground
    bool placeholder;     // the model is not this magazine's own shape
};

const VRMagazineFeed kVRMagazineFeeds[] = {
    {"M16", "misc\\mag_nato.p3d", false},         {"M4", "misc\\mag_nato.p3d", false},
    {"AK47", "misc\\mag_ak.p3d", false},          {"AK74", "misc\\mag_ak.p3d", false},
    {"HK", "misc\\mag_nato.p3d", true},           {"M21", "misc\\mag_nato.p3d", true},
    {"SVDDragunov", "misc\\mag_nato.p3d", true},  {"SteyrMag", "misc\\mag_nato.p3d", true},
    {"G36aMag", "misc\\mag_nato.p3d", true},      {"FALMag", "misc\\mag_nato.p3d", true},
    {"HKG3Mag", "misc\\mag_nato.p3d", true},      {"UZIMag", "misc\\mag_nato.p3d", true},
    {"BizonMag", "misc\\mag_nato.p3d", true},     {"BerettaMag", "misc\\mag_nato.p3d", true},
    {"CZ75Mag", "misc\\mag_nato.p3d", true},      {"GlockMag", "misc\\mag_nato.p3d", true},
    {"GlockSMag", "misc\\mag_nato.p3d", true},    {"IngramMag", "misc\\mag_nato.p3d", true},
    {"SkorpionMag", "misc\\mag_nato.p3d", true},  {"TokarevMag", "misc\\mag_nato.p3d", true},
};

const VRMagazineFeed* FindVRMagazineFeed(const MagazineType* type)
{
    if (!type)
        return nullptr;
    for (const VRMagazineFeed& f : kVRMagazineFeeds)
        if (strcmpi(type->GetName(), f.magazine) == 0)
            return &f;
    return nullptr;
}

// A magazine let go or ejected: falls under gravity, tumbling a little,
// and comes to rest lying flat on the topmost surface below it (terrain or a
// building's floor).  Local and short-lived like cartridge cases (a world
// cloudlet, not saved); at most kMaxDropped stay, each for kLifetime s.
class VRDroppedMagazine : public Vehicle
{
    typedef Vehicle base;

    Vector3 _velocity;
    Vector3 _spin; // rad/s about the model axes
    float _halfThickness;
    float _age = 0.0f;
    bool _resting = false;

  public:
    static constexpr int kMaxDropped = 12;
    static constexpr float kLifetime = 90.0f;

    VRDroppedMagazine(LODShapeWithShadow* shape, const Matrix4& pose, Vector3Par velocity)
        : base(shape, VehicleTypes.New("FxCartridge"), -1), _velocity(velocity)
    {
        Matrix4 m = pose;
        m.Orthogonalize();
        SetTransform(m);
        _halfThickness = shape ? floatMax(-shape->Min().Y(), 0.005f) : 0.012f;
        _spin = Vector3(GRandGen.RandomValue() - 0.5f, GRandGen.RandomValue() - 0.5f, GRandGen.RandomValue() - 0.5f) * 6.0f;
    }

    bool MustBeSaved() const override { return false; }
    float Age() const { return _age; }

    void Simulate(float deltaT, SimulationImportance prec) override
    {
        _age += deltaT;
        if (_age > kLifetime || !GLandscape)
        {
            SetDelete();
            return;
        }
        if (_resting || deltaT <= 0)
            return;

        Matrix4 m = Transform();
        Vector3 pos = m.Position();
        // Topmost surface under the magazine before the move, so it lands on a
        // floor it was above, not on the floor above it.
        const float surface = GLandscape->RoadSurfaceYAboveWater(pos + VUp * 0.05f);
        _velocity[1] -= G_CONST * deltaT;
        pos += _velocity * deltaT;

        m.SetOrientation(m.Orientation() * Matrix3(MRotationX, _spin.X() * deltaT) *
                         Matrix3(MRotationY, _spin.Y() * deltaT) * Matrix3(MRotationZ, _spin.Z() * deltaT));

        if (pos.Y() - _halfThickness <= surface)
        {
            // Rest flat (the model's thin axis up), keeping its heading.
            Vector3 dir = m.Direction();
            dir[1] = 0;
            if (dir.SquareSize() < 1e-4f)
                dir = m.DirectionAside().CrossProduct(VUp);
            Matrix3 flat;
            flat.SetDirectionAndUp(dir.Normalized(), VUp);
            m.SetOrientation(flat);
            pos[1] = surface + _halfThickness;
            _velocity = VZero;
            _resting = true;
            LOG_INFO(Graphics, "VR: dropped magazine at rest after {:.2f} s on the surface at {:.2f} m (terrain {:.2f} m)",
                     _age, surface, GLandscape->SurfaceYAboveWater(pos.X(), pos.Z()));
        }
        m.SetPosition(pos);
        SetTransform(m);
    }
};
} // namespace

bool Man::VRHandReloads(int weapon) const
{
    if (weapon < 0 || weapon >= NMagazineSlots() || !GEngine->VRManualReload() ||
        static_cast<const EntityAI*>(GWorld->PlayerOn()) != this)
        return false;
    const MuzzleType* muzzle = GetMagazineSlot(weapon)._muzzle;
    return muzzle && muzzle->_magazines.Size() > 0 && FindVRMagazineFeed(muzzle->_magazines[0]) != nullptr;
}

LODShapeWithShadow* VRMagazineModel(const MagazineType* type)
{
    const VRMagazineFeed* feed = FindVRMagazineFeed(type);
    if (!feed)
        return nullptr;
    static const MagazineType* s_logged = nullptr;
    if (s_logged != type)
    {
        s_logged = type;
        LOG_INFO(Graphics, "VR: magazine {} shown with {}{}", static_cast<const char*>(type->GetName()), feed->model,
                 feed->placeholder ? " (PLACEHOLDER: no model of this magazine in the game data)" : "");
    }
    return Shapes.New(feed->model, false, false);
}

void Man::VRDropMagazine(const MagazineType* type)
{
    Matrix4 pose;
    Vector3 velocity;
    LODShapeWithShadow* shape = VRMagazineModel(type);
    if (!shape || !GEngine->VRReloadEventPose(pose, velocity))
        return;
    // Keep the count bounded: the oldest dropped magazine goes first.
    int count = 0;
    VRDroppedMagazine* oldest = nullptr;
    for (int i = 0; i < GWorld->NCloudlets(); i++)
    {
        auto* m = dynamic_cast<VRDroppedMagazine*>(GWorld->GetCloudlet(i));
        if (!m || m->ToDelete())
            continue;
        ++count;
        if (!oldest || m->Age() > oldest->Age())
            oldest = m;
    }
    if (count >= VRDroppedMagazine::kMaxDropped && oldest)
        oldest->SetDelete();
    GWorld->AddCloudlet(new VRDroppedMagazine(shape, pose, velocity + Speed()));
}

// VR (M6): the weapon models have their magazine built into the mesh (no
// named selection, no bone).  To show the weapon without its magazine the
// magazine's faces are found by geometry (VRPrepareWeaponMagazine) and
// given their own named selection with its own sections, which can then be
// hidden like the muzzle flash.  The faces are grouped into pieces that
// share vertex positions.
namespace
{
struct MeshPiece
{
    AutoArray<int> faces; // face indices
    Vector3 min, max;
};

void FindMeshPieces(const Shape* shape, AutoArray<MeshPiece>& pieces)
{
    // Weld vertices by position (1 mm), then union faces through shared ones.
    const int nPos = shape->NPos();
    AutoArray<int> weld;
    weld.Resize(nPos);
    for (int i = 0; i < nPos; i++)
    {
        weld[i] = i;
        const Vector3 p = shape->Pos(i);
        for (int j = 0; j < i; j++)
        {
            if ((shape->Pos(j) - p).SquareSize() < 1e-6f)
            {
                weld[i] = weld[j];
                break;
            }
        }
    }
    AutoArray<int> parent;
    parent.Resize(nPos);
    for (int i = 0; i < nPos; i++)
        parent[i] = i;
    auto find = [&](int v)
    {
        while (parent[v] != v)
            v = parent[v] = parent[parent[v]];
        return v;
    };
    shape->BuildFaceIndexToOffset();
    const int nFaces = shape->NFaces();
    for (int f = 0; f < nFaces; f++)
    {
        const Poly& poly = shape->Face(shape->FaceIndexToOffset(f));
        for (int k = 1; k < poly.N(); k++)
        {
            const int a = find(weld[poly.GetVertex(0)]);
            const int b = find(weld[poly.GetVertex(k)]);
            if (a != b)
                parent[a] = b;
        }
    }
    AutoArray<int> pieceOf;
    pieceOf.Resize(nPos);
    for (int i = 0; i < nPos; i++)
        pieceOf[i] = -1;
    for (int f = 0; f < nFaces; f++)
    {
        const Poly& poly = shape->Face(shape->FaceIndexToOffset(f));
        if (poly.N() < 3 || (poly.Special() & (IsHidden | IsHiddenProxy)))
            continue;
        const int root = find(weld[poly.GetVertex(0)]);
        if (pieceOf[root] < 0)
        {
            pieceOf[root] = pieces.Size();
            MeshPiece& p = pieces.Append();
            p.min = Vector3(1e10f, 1e10f, 1e10f);
            p.max = Vector3(-1e10f, -1e10f, -1e10f);
        }
        MeshPiece& p = pieces[pieceOf[root]];
        p.faces.Add(f);
        for (int k = 0; k < poly.N(); k++)
        {
            const Vector3 v = shape->Pos(poly.GetVertex(k));
            for (int a = 0; a < 3; a++)
            {
                p.min[a] = floatMin(p.min[a], v[a]);
                p.max[a] = floatMax(p.max[a], v[a]);
            }
        }
    }
}
} // namespace

// VR (M6): finds the weapon's built-in magazine and makes it hideable.  The
// magazine is the mesh piece whose size matches the magazine's own model
// (sorted extents within 25 % or 1 cm), hanging below the model centre.
// Each visual level where it is found gets a named selection "vr_magazine"
// with its own sections; levels where it is merged with the receiver (or
// drawn as a flat impostor) are logged and left alone.  Returns whether any
// level has it.
bool VRPrepareWeaponMagazine(LODShapeWithShadow* weaponShape, const MagazineType* type)
{
    LODShapeWithShadow* magShape = VRMagazineModel(type);
    if (!weaponShape || !magShape)
        return false;
    auto sortedSize = [](Vector3Par min, Vector3Par max)
    {
        float d[3] = {max.X() - min.X(), max.Y() - min.Y(), max.Z() - min.Z()};
        std::sort(d, d + 3);
        return Vector3(d[0], d[1], d[2]);
    };
    const Vector3 magSize = sortedSize(magShape->Min(), magShape->Max());
    const VBType vbType = weaponShape->GetAllowAnimation() ? VBDynamic : VBStatic;
    bool any = false;
    for (int level = 0; level < weaponShape->NLevels(); level++)
    {
        const float res = weaponShape->Resolution(level);
        Shape* lShape = weaponShape->Level(level);
        if (!lShape || res > 5000.0f || lShape->FindNamedSel("vr_magazine") >= 0)
        {
            any = any || (lShape && lShape->FindNamedSel("vr_magazine") >= 0);
            continue;
        }
        AutoArray<MeshPiece> pieces;
        FindMeshPieces(lShape, pieces);
        int pick = -1;
        for (int i = 0; i < pieces.Size(); i++)
        {
            const MeshPiece& p = pieces[i];
            const Vector3 size = sortedSize(p.min, p.max);
            bool fits = p.max.Y() < 0.03f;
            for (int a = 0; a < 3 && fits; a++)
                fits = fabs(size[a] - magSize[a]) <= floatMax(0.25f * magSize[a], 0.01f);
            if (fits && (pick < 0 || p.faces.Size() > pieces[pick].faces.Size()))
                pick = i;
        }
        if (pick < 0)
        {
            LOG_INFO(Graphics, "VR: {} level {} (res {:.2f}): no separate magazine piece (stays visible)",
                     static_cast<const char*>(weaponShape->GetName()), level, res);
            continue;
        }
        const MeshPiece& p = pieces[pick];
        AutoArray<VertexIndex> faces;
        AutoArray<SelInfo> points;
        AutoArray<bool> used;
        used.Resize(lShape->NPos());
        for (int i = 0; i < used.Size(); i++)
            used[i] = false;
        for (int fi = 0; fi < p.faces.Size(); fi++)
        {
            const int f = p.faces[fi];
            faces.Add(static_cast<VertexIndex>(f));
            const Poly& poly = lShape->Face(lShape->FaceIndexToOffset(f));
            for (int k = 0; k < poly.N(); k++)
                if (!used[poly.GetVertex(k)])
                {
                    used[poly.GetVertex(k)] = true;
                    points.Add(SelInfo(poly.GetVertex(k), 255));
                }
        }
        NamedSelection sel("vr_magazine", points.Data(), points.Size(), faces.Data(), faces.Size());
        sel.SetNeedsSections(true);
        lShape->ReleaseVBuffer();
        lShape->AddNamedSel(sel);
        lShape->FindSections();
        lShape->ConvertToVBuffer(vbType);
        const int index = lShape->FindNamedSel("vr_magazine");
        LOG_INFO(Graphics, "VR: {} level {} (res {:.2f}): magazine = {} faces, {} sections, x {:.3f}..{:.3f} y {:.3f}..{:.3f}",
                 static_cast<const char*>(weaponShape->GetName()), level, res, p.faces.Size(),
                 index >= 0 ? lShape->NamedSel(index).NSections() : -1, p.min.X(), p.max.X(), p.min.Y(), p.max.Y());
        any = true;
    }
    return any;
}

void VRShowWeaponMagazine(LODShapeWithShadow* weaponShape, int level, bool show)
{
    Shape* lShape = weaponShape ? weaponShape->Level(level) : nullptr;
    const int index = lShape ? lShape->FindNamedSel("vr_magazine") : -1;
    if (index < 0)
        return;
    const NamedSelection& sel = lShape->NamedSel(index);
    for (int i = 0; i < sel.NSections(); i++)
    {
        ShapeSection& ss = lShape->GetSection(sel.GetSection(i));
        if (show)
            ss.properties.AndSpecial(~IsHidden);
        else
            ss.properties.OrSpecial(IsHidden);
    }
}

// VR (M6): see SoldierOld.hpp; the gestures are in EngineGL33_VR.cpp.
void Man::VRReloadStep()
{
    // The hand only holds a magazine of the weapon in use.
    if (_currentWeapon < 0 || !VRHandReloads(_currentWeapon))
    {
        _vrHandMag = nullptr;
        return;
    }
    MagazineSlot& current = _magazineSlots[_currentWeapon];
    const MuzzleType* muzzle = current._muzzle;

    // A magazine in the hand that left the inventory (scripts, death) is gone.
    if (_vrHandMag)
    {
        bool owned = false;
        for (int i = 0; i < NMagazines(); i++)
            owned = owned || GetMagazine(i) == _vrHandMag;
        if (!owned)
            _vrHandMag = nullptr;
    }

    auto clearSlots = [&](const Magazine* magazine)
    {
        for (int i = 0; i < _magazineSlots.Size(); i++)
            if (_magazineSlots[i]._magazine == magazine)
                _magazineSlots[i]._magazine = nullptr;
    };

    for (int event = GEngine->VRTakeReloadEvent(); event != Engine::VRReloadNone; event = GEngine->VRTakeReloadEvent())
    {
        Magazine* loaded = current._magazine;
        switch (event)
        {
            case Engine::VRReloadEject: // speed reload: the magazine falls, rounds and all
                if (loaded)
                {
                    LOG_INFO(Graphics, "VR: reload eject, magazine {} with {} rounds dropped",
                             static_cast<const char*>(loaded->_type->GetName()), static_cast<int>(loaded->_ammo));
                    if (loaded->_ammo <= 0)
                        _vrChambered = false; // fired dry: the bolt is locked back
                    Ref<Magazine> keep = loaded;
                    clearSlots(loaded);
                    VRDropMagazine(loaded->_type);
                    RemoveMagazine(loaded);
                    Foundation::GlobalShowMessage(1500, "Magazine dropped (%d rounds)", static_cast<int>(keep->_ammo));
                }
                break;
            case Engine::VRReloadUnload: // tactical: out into the hand, still ours
                if (loaded)
                {
                    LOG_INFO(Graphics, "VR: reload unload, magazine with {} rounds in the hand",
                             static_cast<int>(loaded->_ammo));
                    if (loaded->_ammo <= 0)
                        _vrChambered = false;
                    _vrHandMag = loaded;
                    clearSlots(loaded);
                }
                break;
            case Engine::VRReloadTake: // the fullest spare from the pouch
            {
                Magazine* best = nullptr;
                for (int i = 0; i < NMagazines(); i++)
                {
                    Magazine* m = GetMagazine(i);
                    if (m && m != loaded && m->_ammo > 0 && muzzle->CanUse(m->_type) && (!best || m->_ammo > best->_ammo))
                        best = m;
                }
                _vrHandMag = best;
                LOG_INFO(Graphics, "VR: reload take, {} rounds", best ? static_cast<int>(best->_ammo) : -1);
                break;
            }
            case Engine::VRReloadInsert:
            {
                int index = -1;
                for (int i = 0; i < NMagazines(); i++)
                    if (GetMagazine(i) == _vrHandMag)
                        index = i;
                const bool fits = index >= 0 && muzzle->CanUse(_vrHandMag->_type) && !current._magazine;
                const bool done = fits && ReloadMagazineTimed(_currentWeapon, index, true);
                if (done)
                    PlayReloadMagazineSound(_currentWeapon, muzzle);
                LOG_INFO(Graphics, "VR: reload insert {} ({} rounds)", done ? "done" : "refused",
                         done ? static_cast<int>(current._magazine->_ammo) : 0);
                if (done)
                    Foundation::GlobalShowMessage(1500, _vrChambered ? "Magazine in (%d)"
                                                                     : "Magazine in (%d) - pull the charging handle",
                                                  static_cast<int>(current._magazine->_ammo));
                _vrHandMag = nullptr;
                break;
            }
            case Engine::VRReloadStow:
                LOG_INFO(Graphics, "VR: reload stow ({} rounds back in the pouch)",
                         _vrHandMag ? static_cast<int>(_vrHandMag->_ammo) : -1);
                _vrHandMag = nullptr;
                break;
            case Engine::VRReloadDrop:
                if (_vrHandMag)
                {
                    LOG_INFO(Graphics, "VR: reload drop, magazine with {} rounds falls",
                             static_cast<int>(_vrHandMag->_ammo));
                    Ref<Magazine> dropped = _vrHandMag;
                    _vrHandMag = nullptr;
                    clearSlots(dropped);
                    VRDropMagazine(dropped->_type);
                    RemoveMagazine(dropped);
                }
                break;
            case Engine::VRReloadCharge:
                _vrChambered = true;
                LOG_INFO(Graphics, "VR: reload charge");
                Foundation::GlobalShowMessage(1000, "Charged");
                break;
            default:
                break;
        }
    }

    int spares = 0;
    for (int i = 0; i < NMagazines(); i++)
    {
        const Magazine* magazine = GetMagazine(i);
        if (magazine && magazine != current._magazine && magazine != _vrHandMag && magazine->_ammo > 0 &&
            muzzle->CanUse(magazine->_type))
            ++spares;
    }
    GEngine->VRSetMagazineState(current._magazine != nullptr, spares, _vrChambered, _currentWeapon);
}

int Man::VRArmBones(BlendAnimInfo* out, bool leftArm) const
{
    static const struct
    {
        const char* name;
        float factor;
        bool left;
    } kBones[] = {
        {"pruka", 1.0f, false}, {"ploket", 1.0f, false}, {"pbiceps", 0.5f, false},
        {"lruka", 1.0f, true},  {"lloket", 1.0f, true},  {"lbiceps", 0.5f, true},
    };
    const Skeleton* skeleton = Type()->GetSkeleton();
    int n = 0;
    for (const auto& b : kBones)
    {
        if (b.left != leftArm)
            continue;
        for (int i = 0; skeleton && i < skeleton->NBones(); i++)
        {
            if (strcmpi(skeleton->GetBone(i), b.name) == 0)
            {
                out[n].matrixIndex = i;
                out[n].factor = b.factor;
                ++n;
                break;
            }
        }
    }
    return n;
}

// VR: only the proxy of the weapon in hand (a launcher on the back or a
// holstered pistol stays where the animation puts it), only for the player
// seen from their own eyes.
bool Man::IsVRHeldWeaponProxy(int level, const ProxyObject& proxy) const
{
    // A dead soldier lets go: the weapon falls back to the animation.
    if (!GWorld || GWorld->CameraOn() != this || GWorld->GetCameraType() != CamInternal || _currentWeapon < 0 ||
        IsDead())
        return false;
    const WeaponType* weapon = GetMagazineSlot(_currentWeapon)._weapon;
    if (!weapon)
        return false;
    const int matIndex = GetProxyWeights(level, proxy)[0].GetSel();
    const ManType* type = Type();
    if (weapon->_weaponType & MaskSlotPrimary)
        return matIndex == type->_gunMatIndex;
    if (weapon->_weaponType & MaskSlotSecondary)
        return matIndex == type->_rpgMatIndex;
    if (weapon->_weaponType & MaskSlotHandGun)
        return matIndex == type->_rightHandMatIndex;
    return false;
}

void Man::AnimateMatrix(Matrix4& mat, int level, int selection) const
{
    const AnimationRTWeight& wg = GetSelWeights(level, selection);
    AnimateMatrix(mat, wg);
    int matIndex = wg[0].GetSel();

    if (!_gunTransIdent)
    {
        BLEND_ANIM(aiming);
        const BlendAnimSelections& aimingRes = GetAiming(aiming);

        float aimingFactor = 0;
        for (int bi = 0; bi < aimingRes.Size(); bi++)
        {
            const BlendAnimInfo& blend = aimingRes[bi];
            if (matIndex == blend.matrixIndex)
            {
                aimingFactor = blend.factor;
                break;
            }
        }

        BlendMatrix(mat, _gunTrans, aimingFactor);
    }

    if (!_headTransIdent)
    {
        BLEND_ANIM(head);
        const BlendAnimSelections& headRes = GetHead(head);

        float headFactor = 0;
        for (int bi = 0; bi < headRes.Size(); bi++)
        {
            const BlendAnimInfo& blend = headRes[bi];
            if (matIndex == blend.matrixIndex)
            {
                headFactor = blend.factor;
                break;
            }
        }

        BlendMatrix(mat, _headTrans, headFactor);
    }
}

Vector3 Man::COMPosition() const
{
    return AimingPosition();
}

Vector3 Man::AnimatePoint(int level, int index) const
{
    const ManType* type = Type();
    Vector3 res = VZero;
    const WeightInfo& rtw = type->GetWeights();
    if (_primaryFactor > 0.01f)
    {
        AnimationRT* anim = type->GetAnimation(_primaryMove.id);
        if (anim)
        {
            res = anim->Point(rtw, _shape, level, _primaryTime, index) * _primaryFactor;
        }
    }
    if (_primaryFactor < 0.99f)
    {
        AnimationRT* anim = type->GetAnimation(_secondaryMove.id);
        if (anim)
        {
            res += anim->Point(rtw, _shape, level, _secondaryTime, index) * (1 - _primaryFactor);
        }
    }

    if (type->_aimingAxisPoint >= 0 && !_gunTransIdent)
    {
        BLEND_ANIM(aiming);
        const BlendAnimSelections& aimingRes = GetAiming(aiming);
        AnimationRT::TransformPoint(res, rtw[level], _shape->Level(level), _gunTrans, aimingRes.Data(),
                                    aimingRes.Size(), index);
    }

    BLEND_ANIM(legs);
    const BlendAnimSelections& legsRes = GetLegs(legs);
    AnimationRT::TransformPoint(res, rtw[level], _shape->Level(level), _legTrans, legsRes.Data(), legsRes.Size(),
                                index);

    return res;
}

bool Man::HasFlares(CameraType camType) const
{
    if (camType == CamGunner && _currentWeapon >= 0 && _currentWeapon < NMagazineSlots())
    {
        const MagazineSlot& slot = GetMagazineSlot(_currentWeapon);
        if (slot._muzzle)
        {
            return slot._muzzle->_opticsFlare;
        }
    }
    return base::HasFlares(camType);
}

Matrix4 Man::InsideCamera(CameraType camType) const
{
    int level = _shape->FindMemoryLevel();

    if (camType == CamGunner)
    {
        return GetWeaponRelTransform(_currentWeapon);
    }

    int selIndex = Type()->_pilotPoint;
    if (selIndex < 0)
    {
        return MIdentity;
    }

    const Selection& sel = _shape->LevelOpaque(level)->NamedSel(selIndex);
    Vector3Val pilotPos = AnimatePoint(level, sel[0]);
    Matrix4 transform;

    Vector3 relUp = WorldToModel().DirectionUp();

    transform.SetDirectionAndUp(VForward, relUp);
    transform.SetPosition(pilotPos);
    // calculate recoil camera angle
    if (_recoil)
    {
        _recoil->ApplyRecoil(_recoilTime, transform, 0.2f * _recoilFactor);
        // normalize transform
        transform.Orthogonalize();
    }
    return transform;
}

Vector3 Man::GetCameraDirection(CameraType cam) const
{
    return Direction();
}

Vector3 Man::ExternalCameraPosition(CameraType camType) const
{
    return Type()->_extCameraPosition;
}

Vector3 Man::GetSpeakerPosition() const
{
    return CameraPosition();
}

bool Man::IsVirtual(CameraType camType) const
{
    return true;
}
bool Man::IsVirtualX(CameraType camType) const
{
    auto& input = InputSubsystem::Instance();
    if (input.IsLookAroundEnabled())
    {
        return true;
    }
    return camType != CamInternal && camType != CamExternal;
}
bool Man::IsGunner(CameraType camType) const
{
    return camType == CamGunner || camType == CamInternal || camType == CamExternal;
}

void Man::OverrideCursor(CameraType camType, Vector3& dir) const {}

void Man::LimitCursor(CameraType camType, Vector3& dir) const
{
    if (!QIsManual())
    {
        return;
    }
    if (GetCursorRelMode(camType) != CKeyboard)
    {
        return;
    }

    switch (camType)
    {
        case CamInternal:
        case CamExternal:
        case CamGunner:
            break;
        default:
            return;
    }

    auto& input = InputSubsystem::Instance();
    if (!input.IsLookAroundEnabled())
    {
        float xRot = _gunXRotWanted;
        float yRot = _gunYRotWanted;
        saturate(yRot, Type()->_minGunTurn, Type()->_maxGunTurn);
        saturate(xRot, Type()->_minGunElev, Type()->_maxGunElev);
        Vector3 relDir = (Matrix3(MRotationY, yRot) * Matrix3(MRotationX, -xRot).Direction());
        DirectionModelToWorld(dir, relDir);
    }
}

bool Man::IsCommander(CameraType camType) const
{
    return true;
}
bool Man::ShowAim(int weapon, CameraType camType) const
{
    if (WeaponsDisabled())
    {
        return false;
    }
    if (_laserTargetOn)
    {
        return true;
    }
    if (!ShowWeaponAim())
    {
        return false;
    }
    return camType != CamGunner;
}
bool Man::ShowCursor(int weapon, CameraType camType) const
{
#if _ENABLE_CHEATS
    if (CHECK_DIAG(DECombat))
        return true;
#endif
    return camType != CamGunner;
}

void Man::InitVirtual(CameraType camType, float& heading, float& dive, float& fov) const
{
    if (camType == CamExternal)
    {
        camType = CamInternal;
    }
    base::InitVirtual(camType, heading, dive, fov);
    switch (camType)
    {
        case CamGunner:
            fov = 0.21f;
            dive = 0;
            break;
    }
}

void Man::LimitVirtual(CameraType camType, float& heading, float& dive, float& fov) const
{
    if (camType == CamExternal)
    {
        camType = CamInternal;
    }
    base::LimitVirtual(camType, heading, dive, fov);
    switch (camType)
    {
        case CamGunner:
        {
            int curWeapon = SelectedWeapon();
            if (curWeapon >= 0 && curWeapon < NMagazineSlots())
            {
                const MagazineSlot& slot = GetMagazineSlot(curWeapon);
                const MuzzleType* muzzle = slot._muzzle;
                saturate(fov, muzzle->_opticsZoomMin, muzzle->_opticsZoomMax);
            }
            else
            {
                saturate(fov, 0.21f, 0.21f);
            }
            saturate(heading, 0, 0);
            saturate(dive, -0.7f, +0.8f);
        }
        break;
        case CamInternal:
            break;
    }
}

AnimationRT* MovesType::GetAnimation(MoveId move) const
{
    if (move == MoveIdNone)
    {
        return nullptr;
    }
    return _moves[move];
}

ActionMap* MovesType::GetActionMap(MoveId move) const
{
    if (move == MoveIdNone)
    {
        return nullptr;
    }
    return _moves[move].GetActionMap();
}

const MoveInfo* MovesType::GetMoveInfo(MoveId move) const
{
    if (move == MoveIdNone)
    {
        return nullptr;
    }
    return &_moves[move];
}

MoveId MovesType::GetEquivalent(MoveId move) const
{
    const MoveInfo* info = GetMoveInfo(move);
    if (!info)
    {
        return MoveIdNone;
    }
    MoveId id = info->GetEquivalentTo();
    if (id == MoveIdNone)
    {
        return move;
    }
    return id;
}

bool Man::IsDead() const
{
    return GetActUpDegree() == ManPosDead;
}

bool Man::ShadowPoseFrozen() const
{
    // Mirror the steady-dead-body test the simulation uses to suspend a
    // corpse: dead pose reached, killed and motionless for 5 s. Any later
    // physics push refreshes _lastMovementTime and un-freezes the shadow.
    return IsDead() && _whenKilled != Foundation::Time(0) && _whenKilled < Glob.time - 5 &&
           _lastMovementTime < Glob.time - 5;
}

void Man::SetPrimaryMove(MotionPathItem item)
{
    int oldUD = GetActUpDegree();
    _primaryMove = item;

    int newUD = GetActUpDegree();
    if (oldUD != newUD)
    {
        _upDegreeChangeTime = Glob.time;
    }
}

#define DIAG_QUEUE 0

void Man::RefreshMoveQueue(bool enableVariants)
{ // select random variants
    if (Glob.time >= _variantTime)
    {
        const MoveInfo* stillInfo = Type()->GetMoveInfo(_stillMoveQueueEnd);
        // we may select another move instead of this one
        // playing move long enough - select variant
        if (stillInfo)
        {
            MoveId id = stillInfo->GetEquivalentTo();
            if (enableVariants)
            {
                id = QIsManual() ? stillInfo->RandomVariantPlayer() : stillInfo->RandomVariantAI();
            }
            const MoveInfo* newInfo = Type()->GetMoveInfo(id);
            if (newInfo)
            {
                float wait = newInfo->GetVariantAfter();
                _variantTime = Glob.time + wait;
                MotionPathItem item(id);
#if DIAG_QUEUE
                LOG_DEBUG(Physics, "{}: Switch to variant {} of {} for {:.2f}", DNAME(), NAME_T(Type(), id),
                          NAME_T(Type(), _stillMoveQueueEnd), _variantTime - Glob.time);
#endif
                ChangeMoveQueue(item);
                return;
            }
        }
        _variantTime = Glob.time + 600;
    }
}

bool Man::ChangeMoveQueue(MotionPathItem item)
{
#if DIAG_QUEUE
    LOG_DEBUG(Physics, "ChangeMoveQueue {}", NAME_T(Type(), item.id));
#endif

    if (item.id == _primaryMove.id)
    {
#if DIAG_QUEUE
        LOG_DEBUG(Physics, "{}: Reset queue {}", DNAME(), NAME_T(Type(), item.id));
#endif
        // we are in target state
        if (item.context)
        {
            _primaryMove = item;
        }

        if (_queueMove.Size() > 0)
        {
            _queueMove.Resize(0);
            OnEvent(EEAnimChanged, GetCurrentMove());
        }
        return true;
    }

    const MoveInfo* info = Type()->GetMoveInfo(_primaryMove.id);
    if (info && info->GetTerminal())
    {
        // we are in terminal state - nowhere to change
        return false;
    }

    if (item.id == _secondaryMove.id && _primaryFactor < 1 && !_primaryMove.context)
    {
        const MotionEdge& edge = Type()->Edge(_secondaryMove.id, _primaryMove.id);
        if ((MotionEdgeType)edge.type == MEdgeInterpol)
        {
            swap(_secondaryMove, _primaryMove);
            swap(_secondaryTime, _primaryTime);
            _primaryFactor = 1 - _primaryFactor;
            _queueMove.Resize(0);

            if (item.context)
            {
                _primaryMove.context = item.context;
            }
            OnEvent(EEAnimChanged, GetCurrentMove());
            return true;
        }
    }

    int qSize = _queueMove.Size();
    if (qSize > 0 && _queueMove[qSize - 1].id == item.id)
    {
#if DIAG_QUEUE
        LOG_DEBUG(Physics, "{}: Queue already valid - {}", DNAME(), NAME_T(Type(), item.id));
#endif
        // current path is valid
        return true;
    }

#if DIAG_QUEUE
    LOG_DEBUG(Physics, "{}: Set queue {}", DNAME(), NAME_T(Type(), item.id));
#endif

    // find shortest path from current primary move to target move
    if (_primaryMove.id == MoveIdNone)
    {
        return false;
    }

    bool ret = Type()->FindPath(_queueMove, _primaryMove.id, item);
    if (ret)
    {
        OnEvent(EEAnimChanged, GetCurrentMove());
    }
    return ret;
}

bool Man::SetMoveQueue(MotionPathItem item, bool enableVariants)
{
#if DIAG_QUEUE >= 2
    LOG_DEBUG(Physics, "SetMoveQueue {}", NAME_T(Type(), item.id));
#endif

    if (item.id == MoveIdNone)
    {
        if (item.context)
        {
            ProcessMoveFunction(item.context);
        }

        _queueMove.Resize(0);
        return false;
    }
    MoveId equivItemId = Type()->_moveType->GetEquivalent(item.id);
    MoveId equivStillId = Type()->_moveType->GetEquivalent(_stillMoveQueueEnd);
    if (equivItemId != equivStillId)
    {
        _stillMoveQueueEnd = item.id;
        const MoveInfo* info = Type()->GetMoveInfo(item.id);
        float wait = info->GetVariantAfter();
        _variantTime = Glob.time + wait;

#if DIAG_QUEUE
        LOG_DEBUG(Physics, "{}: Eq move changed - {} (eq {})", DNAME(), NAME_T(Type(), item.id),
                  NAME_T(Type(), equivItemId));
#endif
    }
    else
    {
#if DIAG_QUEUE >= 2
        LOG_DEBUG(Physics, "{}: Same eq move changed - {} (eq {})", DNAME(), NAME_T(Type(), item.id),
                  NAME_T(Type(), _stillMoveQueueEnd));
#endif
    }
    if (Glob.time >= _variantTime)
    {
        const MoveInfo* stillInfo = Type()->GetMoveInfo(_stillMoveQueueEnd);
        MoveId id = stillInfo->GetEquivalentTo();
        if (enableVariants)
        {
            id = QIsManual() ? stillInfo->RandomVariantPlayer() : stillInfo->RandomVariantAI();
        }
        if (id != MoveIdNone)
        {
            const MoveInfo* newInfo = Type()->GetMoveInfo(id);

            float wait = newInfo->GetVariantAfter();
            _variantTime = Glob.time + wait;
            item.id = id;
#if DIAG_QUEUE
            LOG_DEBUG(Physics, "{}: Switch to variant {} of {} for {:.2f}", DNAME(), NAME_T(Type(), id),
                      NAME_T(Type(), equivItemId), _variantTime - Glob.time);
#endif
        }
        else
        {
            _variantTime = Glob.time + 600;
        }
    }
    else
    {
        if (equivItemId == equivStillId)
        {
#if DIAG_QUEUE >= 2
            LOG_DEBUG(Physics, "  equivalent state {} and {}", NAME_T(Type(), item.id),
                      NAME_T(Type(), _stillMoveQueueEnd));
#endif
            if (item.context)
            {
                if (_queueMove.Size() > 0)
                {
                    MotionPathItem& lastItem = _queueMove[_queueMove.Size() - 1];
                    lastItem.context = item.context;
                }
                else
                {
                    ChangeMoveQueue(item);
                    return true;
                }
            }
            return true;
        }
    }
    return ChangeMoveQueue(item);
}

void Man::NextExternalQueue()
{
    if (_externalQueue.Size() > 0)
    {
        _externalMove = _externalQueue[0];
        _externalMoveFinished = false;
        _externalQueue.Delete(0);
    }
}

void Man::AdvanceExternalQueue()
{
    if (_externalMoveFinished || _externalMove.id == MoveIdNone)
    {
        NextExternalQueue();
    }
}

static Object* DirectHitCheck(Man* man, Vector3Par beg, Vector3& hit, const AmmoType* ammo)
{
    CollisionBuffer retVal;
    GLandscape->ObjectCollision(retVal, man, man, beg, hit, 0);
    // seek first hit
    if (retVal.Size() <= 0)
    {
        return nullptr;
    }
    int minI = -1;
    float minT = 1e10;
    for (int i = 0; i < retVal.Size(); i++)
    {
        if (minT > retVal[i].under)
        {
            minI = i;
            minT = retVal[i].under;
        }
    }
    const CollisionInfo& info = retVal[minI];
    hit = info.object->PositionModelToTop(info.pos);
    return info.object;
}

void Man::ThrowGrenadeAction(int weapon)
{
    if (GetNetworkManager().IsControlsPaused())
    {
        return;
    }
    TargetType* target = nullptr;
    const WeaponModeType* mode = GetWeaponMode(weapon);

    if (!mode)
    {
        return;
    }
    const AmmoType* ammo = mode->_ammo;
    if (!ammo)
    {
        return;
    }
    if (ammo->_simulation == AmmoShotStroke)
    {
        float distance = ammo->indirectHitRange * 5 + 0.1f;
        Vector3Val begPos = AimingPosition();
        Vector3 hitPos = begPos + Direction() * distance;
        Object* directHit = DirectHitCheck(this, begPos, hitPos, ammo);
        GLandscape->ExplosionDammage(this, nullptr, directHit, hitPos, Direction(), ammo);
        return;
    }
    bool fired = FireShell(weapon, GetWeaponPoint(weapon), GetWeaponRelDirection(weapon), target);
    if (fired)
    {
        EntityAI::FireWeapon(weapon, target);
        if (_forceFireWeapon == weapon)
        {
            _forceFireWeapon = -1;
        }
    }
}

} // namespace Poseidon
