@[Link("ode_doubled")]
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

  alias NearCallback = Proc(Void*, Geom, Geom, Nil)
  alias HeightfieldGetHeight = Proc(Void*, LibC::Int, LibC::Int)

  enum AllocateFlags
    BasicData = 0
    CollisionData = 0x00000001
    MaskAll = ~0
  end

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
  fun allocate_data = dAllocateODEDataForThread(allocate_flags : LibC::UInt) : LibC::Int
  fun cleanup_all_data = dCleanupODEAllDataForThread
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
  fun body_is_enabled = dBodyIsEnabled(body : Body) : LibC::Int

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
  fun joint_set_hinge2_anchor = dJointSetHinge2Anchor(joint : Joint, x : Real, y : Real, z : Real)
  fun joint_set_hinge2_axis1 = dJointSetHinge2Axis1(joint : Joint, x : Real, y : Real, z : Real)
  fun joint_set_hinge2_axis2 = dJointSetHinge2Axis2(joint : Joint, x : Real, y : Real, z : Real)
  fun joint_set_hinge2_param = dJointSetHinge2Param(joint : Joint, parameter : LibC::Int, value : Real)
  fun joint_add_hinge2_torques = dJointAddHinge2Torques(joint : Joint, torque1 : Real, torque2 : Real)
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
  fun joint_set_a_motor_mode = dJointSetAMotorMode(joint : Joint, mode : LibC::Int)
  fun joint_add_a_motor_torques = dJointAddAMotorTorques(joint : Joint, torque1 : Real, torque2 : Real, torque3 : Real)
  fun joint_set_l_motor_num_axes = dJointSetLMotorNumAxes(joint : Joint, num : LibC::Int)
  fun joint_set_l_motor_axis = dJointSetLMotorAxis(joint : Joint, anum : LibC::Int, rel : LibC::Int, x : Real, y : Real, z : Real)
  fun joint_set_l_motor_param = dJointSetLMotorParam(joint : Joint, parameter : LibC::Int, value : Real)

  fun joint_get_ball_anchor = dJointGetBallAnchor(joint : Joint, result : Vector3)
  fun joint_get_ball_anchor2 = dJointGetBallAnchor2(joint : Joint, result : Vector3)
  fun joint_get_hinge_anchor = dJointGetHingeAnchor(joint : Joint, result : Vector3)
  fun joint_get_hinge_anchor2 = dJointGetHingeAnchor2(joint : Joint, result : Vector3)
  fun joint_get_hinge_axis = dJointGetHingeAxis(joint : Joint, result : Vector3)
  fun joint_get_hinge_param = dJointGetHingeParam(joint : Joint, parameter : LibC::Int) : Real
  fun joint_get_hinge_angle = dJointGetHingeAngle(joint : Joint) : Real
  fun joint_get_hinge_angle_rate = dJointGetHingeAngleRate(joint : Joint) : Real
  fun joint_get_slider_position = dJointGetSliderPosition(joint : Joint) : Real
  fun joint_get_slider_position_rate = dJointGetSliderPositionRate(joint : Joint) : Real
  fun joint_get_slider_axis = dJointGetSliderAxis(joint : Joint, result : Vector3)
  fun joint_get_slider_param = dJointGetSliderParam(joint : Joint, parameter : LibC::Int) : Real
  fun joint_get_hinge_anchor = dJointGetHinge2Anchor(joint : Joint, result : Vector3)
  fun joint_get_hinge_anchor2 = dJointGetHinge2Anchor2(joint : Joint, result : Vector3)
  fun joint_get_hinge2_axis1 = dJointGetHinge2Axis1(joint : Joint, result : Vector3)
  fun joint_get_hinge2_axis2 = dJointGetHinge2Axis2(joint : Joint, result : Vector3)
  fun joint_get_hinge2_param = dJointGetHinge2Param(joint : Joint, parameter : LibC::Int) : Real
  fun joint_get_hinge2_angle1 = dJointGetHinge2Angle1(joint : Joint) : Real
  fun joint_get_hinge2_angle1_rate = dJointGetHinge2Angle1Rate(joint : Joint) : Real
  fun joint_get_hinge2_angle2_rate = dJointGetHinge2Angle2Rate(joint : Joint) : Real
  fun joint_get_universal_anchor = dJointGetUniversalAnchor(joint : Joint, result : Vector3)
  fun joint_get_universal_anchor2 = dJointGetUniversalAnchor2(joint : Joint, result : Vector3)
  fun joint_get_universal_axis1 = dJointGetUniversalAxis1(joint : Joint, result : Vector3)
  fun joint_get_universal_axis2 = dJointGetUniversalAxis2(joint : Joint, result : Vector3)
  fun joint_get_universal_param = dJointGetUniversalParam(joint : Joint, parameter : LibC::Int) : Real
  fun joint_get_universal_angle1 = dJointGetUniversalAngle1(joint : Joint) : Real
  fun joint_get_universal_angle2 = dJointGetUniversalAngle2(joint : Joint) : Real
  fun joint_get_universal_angle1_rate = dJointGetUniversalAngle1Rate(joint : Joint) : Real
  fun joint_get_universal_angle2_rate = dJointGetUniversalAngle2Rate(joint : Joint) : Real
  fun joint_get_a_motor_num_axes = dJointGetAMotorNumAxes(joint : Joint) : LibC::Int
  fun joint_get_a_motor_axis = dJointGetAMotorAxis(joint : Joint, anum : LibC::Int, result : Vector3)
  fun joint_get_a_motor_axis_rel = dJointGetAMotorAxisRel(joint : Joint, anum : LibC::Int) : LibC::Int
  fun joint_get_a_motor_angle = dJointGetAMotorAngle(joint : Joint, anum : LibC::Int) : Real
  fun joint_get_a_motor_angle_rate = dJointGetAMotorAngleRate(joint : Joint, anum : LibC::Int) : Real
  fun joint_get_a_motor_param = dJointGetAMotorParam(joint : Joint, parameter : LibC::Int) : Real
  fun joint_get_a_motor_mode = dJointGetAMotorMode(joint : Joint) : LibC::Int
  fun joint_get_l_motor_num_axes = dJointGetLMotorNumAxes(joint : Joint) : LibC::Int
  fun joint_get_l_motor_axis = dJointGetLMotorAxis(joint : Joint, anum : LibC::Int, result : Vector3)
  fun joint_get_l_motor_param = dJointGetLMotorParam(joint : Joint, parameter : LibC::Int) : Real
  fun joint_set_plane_2d_x_param = dJointSetPlane2DXParam(joint : Joint, parameter : LibC::Int, value : Real)
  fun joint_set_plane_2d_y_param = dJointSetPlane2DYParam(joint : Joint, parameter : LibC::Int, value : Real)
  fun joint_set_plane_2d_angle_param = dJointSetPlane2DAngleParam(joint : Joint, parameter : LibC::Int, value : Real)
  fun joint_get_pr_position = dJointGetPRPosition(joint : Joint) : Real
  fun joint_set_pr_anchor = dJointSetPRAnchor(joint : Joint, x : Real, y : Real, z : Real)
  fun joint_set_pr_axis1 = dJointSetPRAxis1(joint : Joint, x : Real, y : Real, z : Real)
  fun joint_set_pr_axis2 = dJointSetPRAxis2(joint : Joint, x : Real, y : Real, z : Real)
  fun joint_get_pr_anchor = dJointGetPRAnchor(joint : Joint, result : Vector3)
  fun joint_get_pr_axis1 = dJointGetPRAxis1(joint : Joint, result : Vector3)
  fun joint_get_pr_axis2 = dJointGetPRAxis2(joint : Joint, result : Vector3)

  fun joint_set_feedback = dJointSetFeedback(joint : Joint, feedback : JointFeedback*)
  fun joint_get_feedback = dJointGetFeedback(joint : Joint) : JointFeedback*

  fun are_connected = dAreConnected(body1 : Body, body2 : Body) : LibC::Int

  # Mass
  fun mass_set_zero = dMassSetZero(mass : Mass*)
  fun mass_set_parameters = dMassSetParameters(mass : Mass*, themass : Real, 
    cgx : Real, cgy : Real, cgz : Real, 
    i11 : Real, i22 : Real, i33 : Real,
    i12 : Real, i13 : Real, i23 : Real,)
  fun mass_set_sphere = dMassSetSphere(mass : Mass*, density : Real, radius : Real)
  fun mass_set_sphere_total = dMassSetSphereTotal(mass : Mass*, total_mass : Real, radius : Real)
  fun mass_set_capsule = dMassSetCapsule(mass : Mass*, density : Real, direction : LibC::Int, radius : Real, length : Real)
  fun mass_set_capsule_total = dMassSetCapsuleTotal(mass : Mass*, total_mass : Real, direction : LibC::Int, radius : Real, length : Real)
  fun mass_set_cylinder = dMassSetCylinder(mass : Mass*, density : Real, direction : LibC::Int, radius : Real, length : Real)
  fun mass_set_cylinder_total = dMassSetCylinderTotal(mass : Mass*, total_mass : Real, direction : LibC::Int, radius : Real, length : Real)
  fun mass_set_box = dMassSetBox(mass : Mass*, density : Real, lx : Real, ly : Real, lz : Real)
  fun mass_set_box_total = dMassSetBoxTotal(mass : Mass*, density : Real, lx : Real, ly : Real, lz : Real)
  fun mass_adjust = dMassAdjust(mass : Mass*, new_mass : Real)
  fun mass_translate = dMassTranslate(mass : Mass*, x : Real, y : Real, z : Real)
  fun mass_rotate = dMassRotate(mass : Mass*, r : Matrix3)
  fun mass_add = dMassAdd(a : Mass*, b : Mass*)

  # Space
  fun simple_space_create = dSimpleSpaceCreate(space : Space) : Space
  fun hash_space_create = dHashSpaceCreate(space : Space) : Space
  fun quad_tree_space_create = dQuadTreeSpaceCreate(space : Space, center : Vector3, extents : Vector3, depth : LibC::Int) : Space
  fun space_destroy = dSpaceDestroy(space : Space)
  fun space_add = dSpaceAdd(space : Space, geom : Geom)
  fun space_remove = dSpaceRemove(space : Space, geom : Geom)
  fun space_query = dSpaceQuery(space : Space, geom : Geom) : LibC::Int
  fun space_collide = dSpaceCollide(space : Space, data : Void*, callback : NearCallback)
  fun space_collide2 = dSpaceCollide2(o1 : Geom, o2 : Geom, data : Void*, callback : NearCallback)
  fun hash_space_set_levels = dHashSpaceSetLevels(space : Space, min_level : LibC::Int, max_level : LibC::Int)
  fun hash_space_set_levels = dHashSpaceGetLevels(space : Space, min_level : LibC::Int*, max_level : LibC::Int*)
  fun space_set_cleanup = dSpaceSetCleanup(space : Space, mode : LibC::Int)
  fun space_get_cleanup = dSpaceGetCleanup(space : Space) : LibC::Int
  fun space_get_num_geoms = dSpaceGetNumGeoms(space : Space) : LibC::Int
  fun space_get_geom = dSpaceGetGeom(space : Space, i : LibC::Int) : Geom

  # Geom
  fun create_sphere = dCreateSphere(space : Space, radius : Real) : Geom
  fun create_box = dCreateBox(space : Space, lx : Real, ly : Real, lz : Real) : Geom
  fun create_plane = dCreatePlane(space : Space, a : Real, b : Real, c : Real, d : Real)
  fun create_capsule = dCreateCapsule(space : Space, radius : Real, length : Real) : Geom
  fun create_cylinder = dCreateCylinder(space : Space, radius : Real, length : Real) : Geom
  fun create_geom_group = dCreateGeomGroup(space : Space) : Geom

  fun geom_sphere_set_radius = dGeomSphereSetRadius(sphere : Geom, radius : Real)
  fun geom_box_set_lengths = dGeomBoxSetLengths(box : Geom, lx : Real, ly : Real, lz : Real)
  fun geom_plane_set_params = dGeomPlaneSetParams(plane : Geom, a : Real, b : Real, c : Real, d : Real)
  fun geom_capsule_set_params = dGeomCapsuleSetParams(capsule : Geom, radius : Real, length : Real)
  fun geom_cylinder_set_params = dGeomCylinderSetParams(cylinder : Geom, radius : Real, length : Real)
  
  fun geom_sphere_get_radius = dGeomSphereGetRadius(sphere : Geom) : Real
  fun geom_box_get_lengths = dGeomBoxGetLengths(box : Geom, result : Vector3)
  fun geom_plane_get_params = dGeomPlaneGetParams(plane : Geom, result : Vector4)
  fun geom_capsule_get_params = dGeomCapsuleGetParams(capsule : Geom, radius : Real*, length : Real*)
  fun geom_cylinder_get_params = dGeomCylinderGetParams(cylinder : Geom, radius : Real*, length : Real*)
  
  fun geom_sphere_point_depth = dGeomSpherePointDepth(sphere : Geom, x : Real, y : Real, z : Real) : Real
  fun geom_box_point_depth = dGeomBoxPointDepth(box : Geom, x : Real, y : Real, z : Real) : Real
  fun geom_plane_point_depth = dGeomPlanexPointDepth(plane : Geom, x : Real, y : Real, z : Real) : Real
  fun geom_capsule_point_depth = dGeomCapsulePointDepth(capsule : Geom, x : Real, y : Real, z : Real) : Real
  # Says not implemented in ode.pyx
  # fun geom_cylinder_point_depth = dGeomCylinderPointDepth(cylinder : Geom, x : Real, y : Real, z : Real) : Real

  fun create_ray = dCreateRay(space : Space, legnth : Real) : Geom
  fun geom_ray_set_length = dGeomRaySetLength(ray : Geom, length : Real)
  fun geom_ray_get_length = dGeomRayGetLength(ray : Geom) : Real
  fun geom_ray_set = dGeomRaySet(ray : Geom, px : Real, py : Real, pz : Real, dx : Real, dy : Real, dz : Real)
  fun geom_ray_get = dGeomRayGet(ray : Geom, start : Vector3, dir : Vector3)
  fun geom_set_data = dGeomSetData(geom : Geom, data : Void*)
  fun geom_get_data = dGeomGetData(geom : Geom) : Void*
  fun geom_set_body = dGeomSetBody(geom : Geom, body : Body)
  fun geom_get_body = dGeomGetBody(geom : Geom) : Body
  fun geom_set_position = dGeomSetPosition(geom : Geom, x : Real, y : Real, z : Real)
  fun geom_set_rotation = dGeomSetRotation(geom : Geom, r : Matrix3)
  fun geom_set_quaternion = dGeomSetQuaternion(geom : Geom, q : Quaternion)
  fun geom_get_position = dGeomGetPosition(geom : Geom) : Real*
  fun geom_get_rotation = dGeomGetRotation(geom : Geom) : Real*
  fun geom_get_quaternion = dGeomGetQuaternion(geom : Geom, result : Quaternion)
  fun geom_set_offset_position = dGeomSetOffsetPosition(geom : Geom, x : Real, y : Real, z : Real)
  fun geom_set_offset_rotation = dGeomSetOffsetRotation(geom : Geom, r : Matrix3)
  fun geom_clear_offset = dGeomClearOffset(geom : Geom)
  fun geom_get_offset_position = dGeomGetOffsetPosition(geom : Geom) : Real*
  fun geom_get_offset_rotation = dGeomGetOffsetRotation(geom : Geom) : Real*
  fun geom_destroy = dGeomDestroy(geom : Geom)
  fun geom_get_aabb = dGeomGetAABB(geom : Geom, aabb : StaticArray(Real, 6))
  fun geom_get_space_aabb = dGeomGetSpaceAABB(geom : Geom) : Real*
  fun geom_is_space = dGeomIsSpace(geom : Geom) : LibC::Int
  fun geom_get_space = dGeomGetSpace(geom : Geom) : Space
  fun geom_get_class = dGeomGetClass(geom : Geom) : LibC::Int
  
  fun geom_set_category_bits = dGeomSetCategoryBits(geom : Geom, bits : LibC::ULong)
  fun geom_set_collide_bits = dGeomSetCollideBits(geom : Geom, bits : LibC::ULong)
  fun geom_get_category_bits = dGeomGetCategoryBits(geom : Geom) : LibC::ULong
  fun geom_get_collide_bits = dGeomGetCollideBits(geom : Geom) : LibC::ULong
  
  fun geom_enable = dGeomEnable(geom : Geom)
  fun geom_disable = dGeomDisable(geom : Geom)
  fun geom_is_enabled = dGeomIsEnabled(geom : Geom) : LibC::Int

  fun geom_group_add = dGeomGroupAdd(group : Geom, x : Geom)
  fun geom_group_remove = dGeomGroupRemove(group : Geom, x : Geom)
  fun geom_group_get_num_geoms = dGeomGroupGetNumGeoms(geom : Geom) : LibC::Int
  fun geom_group_get_geom = dGeomGroupGetGeom(group : Geom, i : LibC::Int) : Geom

  fun create_geom_transform = dCreateGeomTransform(space : Space) : Geom
  fun geom_transform_set_geom = dGeomTransformSetGeom(g : Geom, obj : Geom)
  fun geom_transform_get_geom = dGeomTransformGetGeom(g : Geom) : Geom
  fun geom_transform_set_cleanup = dGeomTransformSetCleanup(g : Geom, mode : LibC::Int)
  fun geom_transform_get_cleanup = dGeomTransformGetCleanup(g : Geom) : LibC::Int
  fun geom_transform_set_info = dGeomTransformSetInfo(g : Geom, mode : LibC::Int)
  fun geom_transform_get_info = dGeomTransformGetInfo(g : Geom) : LibC::Int
  

  fun collide = dCollide(o1 : Geom, o2 : Geom, flags : LibC::Int, contact : ContactGeom*, skip : LibC::Int) : LibC::Int
  
  # Trimesh
  fun geom_tri_mesh_data_create = dGeomTriMeshDataCreate : TriMeshData
  fun geom_tri_mesh_data_destroy = dGeomTriMeshDataDestroy(g : TriMeshData)
  fun geom_tri_mesh_data_build_single1 = dGeomTriMeshDataBuildSingle1(
    g : TriMeshData, vertices : Void*, vertex_stride : LibC::Int,
    vertex_count : LibC::Int, indices : Void*, index_count : LibC::Int,
    tri_stride : LibC::Int, normals : Void*
    )
  fun geom_tri_mesh_data_build_simple = dGeomTriMeshDataBuildSimple(g : TriMeshData, vertices : Real*, vertex_count : LibC::Int, indices : LibC::Int*, index_count : LibC::Int)
  fun create_tri_mesh = dCreateTriMesh(space : Space, data : TriMeshData, callback : Void*, array_callback : Void*, ray_callback : Void*) : Geom
  fun geom_tri_mesh_set_data = dGeomTriMeshSetData(g : Geom, data : TriMeshData)
  fun geom_tri_mesh_clear_tc_cache = dGeomTriMeshClearTCCache(geom : Geom)
  fun geom_tri_mesh_get_triangle = dGeomTriMeshGetTriangle(g : Geom, index : LibC::Int, v0 : Vector3*, v1 : Vector3*, v2 : Vector3*)
  fun geom_tri_mesh_get_triangle_count = dGeomTriMeshGetTriangleCount(g : Geom) : LibC::Int
  fun geom_tri_mesh_get_point = dGeomTriMeshGetPoint(g : Geom, index : LibC::Int, u : Real, v : Real, out_v : Vector3)
  fun geom_tri_mesh_enable_tc = dGeomTriMeshEnableTC(g : Geom, geom_class : LibC::Int, enable : LibC::Int)
  fun geom_tri_mesh_is_tc_enabled = dGeomTriMeshIsTCEnabled(g : Geom, geom_class : LibC::Int) : LibC::Int
  
  # Heightfield
  fun geom_heightfield_data_create = dGeomHeightfieldDataCreate : HeightfieldData 
  fun geom_heightfield_data_destroy = dGeomHeightfieldDataDestroy(g : HeightfieldData)

  fun geom_heightfield_build_callback = dGeomHeightfieldDataBuildCallback(
    d : HeightfieldData, user_data : Void*, callback : HeightfieldGetHeight*,
    width : Real, height : Real, depth : Real, width_samples : LibC::Int,
    scale : Real, offset : Real, thickness : Real, b_wrap : LibC::Int
  )
  fun create_heightfield = dCreateHeightfield(space : Space, data : HeightfieldData, b_placeable : LibC::Int) : Geom
end