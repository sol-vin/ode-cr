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

  # World
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

  fun world_get_contact_surface_layer = dWorldGetContactSurfaceLayer(world : World) : Real
  fun world_set_contact_surface_layer = dWorldSetContactSurfaceLayer(world : World, depth : Real)


  fun world_set_auto_disable_flag = dWorldSetAutoDisableFlag(world : World, do_auto_disable : LibC::Int)
  fun world_get_auto_disable_flag = dWorldGetAutoDisableFlag(world : World) : LibC::Int

  fun world_get_auto_disable_linear_threshold = dWorldGetAutoDisableLinearThreshold(world : World) : Real
  fun world_set_auto_disable_linear_threshold = dWorldSetAutoDisableLinearThreshold(world : World, linear_threshold : Real)
  
  fun world_get_auto_disable_angular_threshold = dWorldGetAutoDisableAngularThreshold(world : World) : Real
  fun world_set_auto_disable_angular_threshold = dWorldSetAutoDisableAngularThreshold(world : World, angular_threshold : Real)

  fun world_set_auto_disable_steps = dWorldSetAutoDisableSteps(world : World, steps : LibC::Int)
  fun world_get_auto_disable_steps = dWorldGetAutoDisableSteps(world : World) : LibC::Int

  fun world_get_auto_disable_time = dWorldGetAutoDisableTime(world : World) : Real
  fun world_set_auto_disable_time = dWorldSetAutoDisableTime(world : World, time : Real)

  fun world_get_linear_damping = dWorldGetLinearDamping(world : World) : Real
  fun world_set_linear_damping = dWorldSetLinearDamping(world : World, scale : Real)

  fun world_get_angular_damping = dWorldGetAngularDamping(world : World) : Real
  fun world_set_angular_damping = dWorldSetAngularDamping(world : World, scale : Real)

  fun world_impulse_to_force = dWorldImpulseToForce(world : World, step_size : Real, ix : Real, iy : Real, iz : Real, force : Vector3)

  # Body
  fun body_create = dBodyCreate(world : World) : Body
  fun body_destroy = dBodyDestroy(body : Body)

  fun body_set_data = dBodySetData(body : Body, data : Void*)
  fun body_get_data = dBodyGetData(body : Body) : Void*

  fun body_set_position = dBodySetPosition(body : Body, x : Real, y : Real, z : Real)
  fun body_set_rotation = dBodySetRotation(body : Body, r : Matrix3)
  fun body_set_quaternion = dBodySetQuaternion(body : Body, q : Quaternion)
  fun body_set_linear_vel = dBodySetLinearVel(body : Body, x : Real, y : Real, z : Real)
  fun body_set_angular_vel = dBodySetAngularVel(body : Body, x : Real, y : Real, z : Real)

  fun body_get_position = dBodyGetPosition(body : Body) : Real*
  fun body_get_rotation = dBodyGetRotation(body : Body) : Real*
  fun body_get_quaternion = dBodyGetQuaternion(body : Body) : Real*
  fun body_get_linear_vel = dBodyGetLinearVel(body : Body) : Real*
  fun body_get_angular_vel = dBodyGetAngularVel(body : Body) : Real*

  fun body_set_mass = dBodySetMass(body : Body, mass : Mass*)
  fun body_get_mass = dBodyGetMass(body : Body, mass : Mass*)
  
  fun body_add_force = dBodyAddForce(body : Body, fx : Real, fy : Real, fz : Real)
  fun body_add_torque = dBodyAddTorque(body : Body, fx : Real, fy : Real, fz : Real)
  fun body_add_rel_force = dBodyAddRelForce(body : Body, fx : Real, fy : Real, fz : Real)
  fun body_add_rel_torque = dBodyAddRelTorque(body : Body, fx : Real, fy : Real, fz : Real)
  fun body_add_force_at_pos = dBodyAddForceAtPos(body : Body, fx : Real, fy : Real, fz : Real, px : Real, py : Real, pz : Real)
  fun body_add_force_at_rel_pos = dBodyAddForceAtRelPos(body : Body, fx : Real, fy : Real, fz : Real, px : Real, py : Real, pz : Real)
  fun body_add_rel_force_at_pos = dBodyAddRelForceAtPos(body : Body, fx : Real, fy : Real, fz : Real, px : Real, py : Real, pz : Real)
  fun body_add_rel_force_at_rel_pos = dBodyAddRelForceAtRelPos(body : Body, fx : Real, fy : Real, fz : Real, px : Real, py : Real, pz : Real)

  fun body_get_force = dBodyGetForce(body : Body) : Real*
  fun body_get_torque = dBodyGetTorque(body : Body) : Real*

  fun body_set_force = dBodySetForce(body : Body, x : Real, y : Real, z : Real)
  fun body_set_torque = dBodySetTorque(body : Body, x : Real, y : Real, z : Real)

  fun body_get_rel_point_pos = dBodyGetRelPointPos(body : Body, px : Real, py : Real, pz : Real, result : Vector3)
  fun body_get_rel_point_vel = dBodyGetRelPointVel(body : Body, px : Real, py : Real, pz : Real, result : Vector3)
  
  fun body_get_point_vel = dBodyGetPointVel(body : Body, px : Real, py : Real, pz : Real, result : Vector3)
  fun body_get_pos_rel_point = dBodyGetPosRelPoint(body : Body, px : Real, py : Real, pz : Real, result : Vector3)

  fun body_vector_to_world = dBodyVectorToWorld(body : Body, px : Real, py : Real, pz : Real, result : Vector3)
  fun body_vector_from_world = dBodyVectorFromWorld(body : Body, px : Real, py : Real, pz : Real, result : Vector3)

  fun body_set_finite_rotation_mode = dBodySetFiniteRotationMode(body : Body, mode : LibC::Int)
  fun body_set_finite_rotation_axis = dBodySetFiniteRotationAxis(body : Body, x : Real, y : Real, z : Real)

  fun body_get_finite_rotation_mode = dBodyGetFiniteRotationMode(body : Body) : LibC::Int
  fun body_get_finite_rotation_axis = dBodyGetFiniteRotationAxis(body : Body, result : Vector3)
  
  fun body_get_num_joints = dBodyGetNumJoints(body : Body) : LibC::Int
  fun body_get_num_joints = dBodyGetJoint(body : Body, index : LibC::Int) : Joint

  fun body_enable = dBodyEnable(body : Body)
  fun body_disable = dBodyDisable(body : Body)
  fun body_is_enabled = dBodyEnable(body : Body) : LibC::Int

  fun body_set_gravity_mode = dBodySetGravityMode(body : Body, mode : LibC::Int)
  fun body_get_gravity_mode = dBodyGetGravityMode(body : Body) : LibC::Int
  fun body_set_dynamic = dBodySetDynamic(body : Body)
  fun body_set_kinematic = dBodySetKinematic(body : Body)
  fun body_is_kinematic = dBodyIsKinematic(body : Body) : LibC::Int
  fun body_set_max_angular_speed = dBodySetMaxAngularSpeed(body : Body, max_speed : Real)

  # Joints
  fun joint_create_ball = dJointCreateBall(world : World, joint_group : JointGroup) : Joint
  fun joint_create_hinge = dJointCreateHinge(world : World, joint_group : JointGroup) : Joint
  fun joint_create_slider = dJointCreateSlider(world : World, joint_group : JointGroup) : Joint
  fun joint_create_contact = dJointCreateContact(world : World, joint_group : JointGroup, contact : Contact*) : Joint
  fun joint_create_universal = dJointCreateUniversal(world : World, joint_group : JointGroup) : Joint
  fun joint_create_pr = dJointCreatePR(world : World, joint_group : JointGroup) : Joint
  fun joint_create_hinge2 = dJointCreateHinge2(world : World, joint_group : JointGroup) : Joint
  fun joint_create_fixed = dJointCreateFixed(world : World, joint_group : JointGroup) : Joint
  fun joint_create_null = dJointCreateNull(world : World, joint_group : JointGroup) : Joint
  fun joint_create_a_motor = dJointCreateAMotor(world : World, joint_group : JointGroup) : Joint
  fun joint_create_l_motor = dJointCreateLMotor(world : World, joint_group : JointGroup) : Joint
  fun joint_create_plane_2d = dJointCreatePlane2D(world : World, joint_group : JointGroup) : Joint
  
  fun joint_destroy = dJointDestroy(joint : Joint)
  fun joint_enable = dJointEnable(joint : Joint)
  fun joint_disable = dJointDisable(joint : Joint)
  fun joint_is_enabled = dJointIsEnabled(joint : Joint) : LibC::Int
  
  fun joint_group_create = dJointGroupCreate(max_size : LibC::Int) : JointGroup
  fun joint_group_destroy = dJointGroupDestroy(joint : JointGroup)
  fun joint_group_empty = dJointGroupEmpty(joint : JointGroup)
  
  fun joint_attach = dJointAttach(joint : Joint, body1 : Body, body2 : Body)
  fun joint_set_data = dJointSetData(joint : Joint, data : Void*)
  fun joint_get_data = dJointGetData(joint : Joint) : Void*
  fun joint_get_type = dJointGetType(joint : Joint) : LibC::Int
  fun joint_get_body = dJointGetBody(joint : Joint, index : LibC::Int) : Body


  fun joint_set_ball_anchor = dJointSetBallAnchor(joint : Joint, x : Real, y : Real, z : Real)
  fun joint_set_hinge_anchor = dJointSetHingeAnchor(joint : Joint, x : Real, y : Real, z : Real)
  fun joint_set_hinge_axis = dJointSetHingeAxis(joint : Joint, x : Real, y : Real, z : Real)
  fun joint_set_hinge_param = dJointSetHingeParam(joint : Joint, parameter : LibC::Int, value : Real)
  fun joint_add_hinge_torque = dJointAddHingeTorque(joint : Joint, torque : Real)
  fun joint_set_slider_axis = dJointSetSliderAxis(joint : Joint, x : Real, y : Real, z : Real)
  fun joint_set_slider_param = dJointSetSliderParam(joint : Joint, parameter : LibC::Int, value : Real)
  fun joint_add_slider_force = dJointAddSliderForce(joint : Joint, force : Real)
  fun joint_set_hinge_2_anchor = dJointSetHinge2Anchor(joint : Joint, x : Real, y : Real, z : Real)
  fun joint_set_hinge_2_axis1 = dJointSetHinge2Axis1(joint : Joint, x : Real, y : Real, z : Real)
  fun joint_set_hinge_2_axis2 = dJointSetHinge2Axis2(joint : Joint, x : Real, y : Real, z : Real)
  fun joint_set_hinge_2_param = dJointSetHinge2Param(joint : Joint, parameter : LibC::Int, value : Real)
  fun joint_add_hinge_2_torques = dJointAddHinge2Torques(joint : Joint, torque1 : Real, torque2 : Real)
  fun joint_set_universal_anchor = dJointSetUniversalAnchor(joint : Joint, x : Real, y : Real, z : Real)
  fun joint_set_universal_axis1 = dJointSetUniversalAxis1(joint : Joint, x : Real, y : Real, z : Real)
  fun joint_set_universal_axis2 = dJointSetUniversalAxis2(joint : Joint, x : Real, y : Real, z : Real)
  fun joint_set_universal_param = dJointSetUniversalParam(joint : Joint, parameter : LibC::Int, value : Real)
  fun joint_add_universal_torques = dJointAddUniversalTorques(joint : Joint, torque1 : Real, torque2 : Real)
  fun joint_set_fixed = dJointSetFixed(joint : Joint)
  fun joint_set_a_motor_num_axes = dJointSetAMotorNumAxes(joint : Joint, num : LibC::Int)
  fun joint_set_a_motor_axis = dJointSetAMotorAxis(joint : Joint, anum : LibC::Int, rel : LibC::Int, x : Real, y : Real, z : Real)
  fun joint_set_a_motor_angle = dJointSetAMotorAngle(joint : Joint, anum : LibC::Int, angle : Real)
  fun joint_set_a_motor_param = dJointSetAMotorParam(joint : Joint, parameter : LibC::Int, value : Real)
  fun joint_set_a_motor_mode = dJointSetAMotorMode(joint : Joint, mode :: LibC::Int)
  fun joint_add_a_motor_torques = dJointAddAMotorTorques(joint : Joint, torque1 : Real, torque2 : Real, torque3 : Real)
  fun joint_set_l_motor_num_axes = dJointSetLMotorNumAxes(joint : Joint, num : LibC::Int)
  fun joint_set_l_motor_axis = dJointSetLMotorAxis(joint : Joint, anum : LibC::Int, rel : LibC::Int, x : Real, y : Real, z : Real)
  fun joint_set_l_motor_param = dJointSetLMotorParam(joint : Joint, parameter : LibC::Int, value : Real)
end