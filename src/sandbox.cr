require "./ode-cr"

puts "Hi"
ODE.init

10_000.times do
  world = ODE.world_create
  ODE.world_set_gravity(world, StaticArray[0.0, 1.0, 0.0, 0.0])

  ODE.world_destroy world
end

ODE.close