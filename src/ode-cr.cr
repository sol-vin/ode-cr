@[Link("ode")]
lib ODE
  DUMMY = [:World, :Space, :Body, :Geom, :Joint, :JointGroup, :TriMeshData, :HeightfieldData]

  {% for name in DUMMY %}
  struct {{name.id}}
    this : Void*
  end
  {% end %}

  alias Real = LibC::Double
  alias Vector3 = StaticArray(Real, 4)
  alias Vector4 = StaticArray(Real, 4)
  alias Matrix3 = StaticArray(Real, 12)
  alias Matrix4 = StaticArray(Real, 16)
  alias Matrix6 = StaticArray(Real, 48)
  alias Quaternion = StaticArray(Real, 4)

  alias NearCallback = Proc(Void*, Geom, Geom)
  alias HeightfieldGetHeight = Proc(Void*, LibC::Int, LibC::Int)

  struct Mass
    mass : Real
    c : Vector4
    i : Matrix3
  end

  struct JointFeedback
    f1 : Vector3
    t1 : Vector3
    f2 : Vector3
    t2 : Vector3
  end

  struct SurfaceParameters
    mode : LibC::Int
    mu : Real

    mu2 : Real
    bounce : Real
    bounce_vel : Real
    soft_erp : Real
    soft_cfm : Real
    motion1 : Real
    motion2 : Real
    slip1 : Real
    slip2 : Real
  end

  struct ContactGeom
    pos : Vector3
    normal : Vector3
    depth : Real
    g1 : Geom
    g2 : Geom
  end

  struct Contact
    surface : SurfaceParameters
    geom : ContactGeom
    fdir1 : Vector3
  end

  fun init = dInitODE
  #fun init2 = dInitODE2(flags : LibC::UInt)
  fun close = dCloseODE

  fun world_create = dWorldCreate : World
  fun world_destroy = dWorldDestroy(world : World)

  fun world_set_gravity = dWorldSetGravity(world : World, x : Real, y : Real, z : Real)

  fun world_get_erp = dWorldGetERP(world : World) : Real
  fun world_set_erp = dWorldSetERP(world : World, erp : Real)
  fun world_get_cfm = dWorldGetCFM(world : World) : Real
  fun world_set_cfm = dWorldSetCFM(world : World, cfm : Real)

  fun world_step = dWorldStep(world : World, stepsize : Real)
  fun world_quick_step = dWorldQuickStep(world : World, stepsize : Real)

  fun world_set_quick_step_num_iterations = dWorldSetQuickStepNumIterations(world : World, num : LibC::Int)
  fun world_get_quick_step_num_iterations = dWorldGetQuickStepNumIterations(world : World) : LibC::Int

  fun world_get_contact_max_correcting_velocity = dWorldGetContactMaxCorrectingVel(world : World) : Real
  fun world_set_contact_max_correcting_velocity = dWorldSetContactMaxCorrectingVel(world : World, vel : Real)
end