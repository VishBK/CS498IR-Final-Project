extends KinematicBody

onready var body = $body
onready var armA = $body/armA
onready var armB = $body/armB
onready var handA = $body/armA/handA
onready var handB = $body/armB/handB

# Declare member variables here. Examples:
# var a = 2
# var b = "text"
#var gravity = Vector3.DOWN * 12
var speed = 0.01
var velocity = Vector3()
#var len_limb = 0

# Called when the node enters the scene tree for the first time.
func _ready():
	print("hello")
	#len_limb = handA.position.y * -1

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(delta):
	velocity.x += speed
	velocity = move_and_slide(velocity, Vector3(1, 1, 1))
	
	armA.rotate_x(-0.1 * delta)
	armB.rotate_x(0.1 * delta)
	handA.rotate_x(0.1 * delta)
	handB.rotate_x(-0.1 * delta)
	
	if body.translation.x < -1 or body.translation.x > 1:
		speed *= -1
	
	#print(body.get_translation())
	print(armA.rotation)
	
