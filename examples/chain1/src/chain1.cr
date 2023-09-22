require "ode-cr"
require "raylib-cr"

module Chain1
  alias R = Raylib
  alias RV3 = Raylib::Vector3
  alias O = ODE

  module Screen
    WIDTH  = 1200
    HEIGHT =  800
  end

  NUM    =     10
  SIDE   =    0.2
  MASS   =    1.0
  RADIUS = 0.1732

  class_getter world : O::World = O::World.new
  class_getter space : O::Space = O::Space.new
  class_getter bodies : Array(O::Body) = [] of O::Body
  class_getter joints : Array(O::Joint) = [] of O::Joint
  class_getter contact_group : O::JointGroup = O::JointGroup.new
  class_getter chain_links : Array(O::Geom) = [] of O::Geom
  class_getter angle = 0.0

  @@link_mass = O::Mass.new

  class_getter camera = R::Camera3D.new(
    position: RV3.unit_y * 2,
    up: RV3.new(y: 1), fovy: 70.0,
    target: RV3.new(x: 1),
    projection: R::CameraProjection::Perspective)

  CALLBACK = ->(data : Void*, o1 : O::Geom, o2 : O::Geom) do
    contact = O::Contact.new
    geom = contact.geom

    b1 = O.geom_get_body(o1)
    b2 = O.geom_get_body(o2)

    return if O.are_connected(b1, b2) > 0

    contact.surface.mu = 0.1

    if O.collide(o1, o2, 1, pointerof(geom), sizeof(O::ContactGeom)) > 0
      c = O.joint_create_contact(world, contact_group, pointerof(contact))
      O.joint_attach(c, b1, b2)
    end
  end

  def self.step
    @@angle += 0.05
    O.body_add_force(bodies.last, 0, 0, 1.5*Math.sin(angle) + 1.0)
    # O.space_collide(space, Pointer(Void*).null, pointerof(CALLBACK))
    O.world_step(world, 0.05)
    O.joint_group_empty(contact_group)
  end

  def self.update
    @@camera.target = o_to_r(O.body_get_position(bodies.first))
    step
  end

  def self.draw
    R.begin_drawing
    R.clear_background R::WHITE
    R.begin_mode_3d @@camera
    bodies.each do |b|
      R.draw_plane(R::Vector3.zero, R::Vector2.one*100, R::BLUE)
      R.draw_sphere(o_to_r(O.body_get_position(b)), RADIUS, R::RED)
    end
    R.end_mode_3d
    R.end_drawing
  end

  # Changes a ODE::Vector3 to  a Raylib::Vector3
  private def self.o_to_r(o_pos : ODE::Real*) : R::Vector3
    R::Vector3.new(
      x: o_pos[0],
      y: o_pos[1],
      z: o_pos[2],
    )
  end

  def self.run
    # Setup Raylib
    R.init_window(Screen::WIDTH, Screen::HEIGHT, "wireland")
    R.set_target_fps(60)
    O.allocate_data(O::AllocateFlags::MaskAll)
    O.init

    # Setup physics
    @@link_mass = O::Mass.new
    @@world = O.world_create
    @@space = O.hash_space_create(O::Space.new)
    @@contact_group = O.joint_group_create(1000000)
    O.world_set_gravity(world, 0, 0, -0.5)
    O.create_plane(space, 0, 0, 1, 0)

    NUM.times do |i|
      bodies << O.body_create(world)
      k = i*SIDE

      O.body_set_position(bodies[i], k, k, k + 0.4)
      O.mass_set_box(pointerof(@@link_mass), 1, SIDE, SIDE, SIDE)
      O.mass_adjust(pointerof(@@link_mass), MASS)
      O.body_set_mass(bodies[i], pointerof(@@link_mass))
      chain_links << O.create_sphere(space, RADIUS)
      O.geom_set_body(chain_links[i], bodies[i])
    end

    (NUM - 1).times do |i|
      joints << O.joint_create_ball(world, O::JointGroup.new)
      O.joint_attach(joints[i], bodies[i], bodies[i + 1])
      k = (i + 0.5)*SIDE
      O.joint_set_ball_anchor(joints[i], k, k, k + 0.4)
    end

    until R.close_window?
      update
      draw
    end

    R.close_window
    O.close
  end
end

Chain1.run
