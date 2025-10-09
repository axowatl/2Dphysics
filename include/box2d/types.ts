import { b2_lengthUnitsPerMeter } from "../src/core.ts";
import { b2Rot, b2Rot_identity, b2Transform, b2Vec2 } from "./math_functions.ts";
import { b2BodyId, b2ContactId, b2JointId, b2ShapeId } from "./id.ts";

export const B2_DEFAULT_CATEGORY_BITS: bigint = 1n;
export const B2_DEFAULT_MASK_BITS: bigint = 2n ** 64n - 1n;
export const B2_SECRET_COOKIE: number = 1152023;

/// Task interface
/// This is prototype for a Box2D task. Your task system is expected to invoke the Box2D task with these arguments.
/// The task spans a range of the parallel-for: [startIndex, endIndex)
/// The worker index must correctly identify each worker in the user thread pool, expected in [0, workerCount).
/// A worker must only exist on only one thread at a time and is analogous to the thread index.
/// The task context is the context pointer sent from Box2D when it is enqueued.
/// The startIndex and endIndex are expected in the range [0, itemCount) where itemCount is the argument to b2EnqueueTaskCallback
/// below. Box2D expects startIndex < endIndex and will execute a loop like this:
///
/// @code{.c}
/// for (int i = startIndex; i < endIndex; ++i)
/// {
/// 	DoWork();
/// }
/// @endcode
/// @ingroup world
export type b2TaskCallback = (
	startIndex: number,
	endIndex: number,
	workerIndex: number,
	taskContext: void
) => void;

/// These functions can be provided to Box2D to invoke a task system. These are designed to work well with enkiTS.
/// Returns a pointer to the user's task object. May be nullptr. A nullptr indicates to Box2D that the work was executed
/// serially within the callback and there is no need to call b2FinishTaskCallback.
/// The itemCount is the number of Box2D work items that are to be partitioned among workers by the user's task system.
/// This is essentially a parallel-for. The minRange parameter is a suggestion of the minimum number of items to assign
/// per worker to reduce overhead. For example, suppose the task is small and that itemCount is 16. A minRange of 8 suggests
/// that your task system should split the work items among just two workers, even if you have more available.
/// In general the range [startIndex, endIndex) send to b2TaskCallback should obey:
/// endIndex - startIndex >= minRange
/// The exception of course is when itemCount < minRange.
/// @ingroup world
export type b2EnqueueTaskCallback = (
	task: b2TaskCallback,
	itemCount: number,
	minRange: number,
	taskContext: void,
	userContext: void
) => void;

/// Finishes a user task object that wraps a Box2D task.
/// @ingroup world
export type b2FinishTaskCallback = (
	userTask: void,
	userContext: void
) => void;

/// Optional friction mixing callback. This intentionally provides no context objects because this is called
/// from a worker thread.
/// @warning This function should not attempt to modify Box2D state or user application state.
/// @ingroup world
export type b2FrictionCallback = (
  frictionA: number,
  userMaterialIdA: bigint,
  frictionB: number,
  userMaterialIdB: bigint
) => number;

/// Optional restitution mixing callback. This intentionally provides no context objects because this is called
/// from a worker thread.
/// @warning This function should not attempt to modify Box2D state or user application state.
/// @ingroup world
export type b2RestitutionCallback = (
	restitutionA: number,
	userMaterialIdA: bigint,
	restitutionB: number,
	userMaterialIdB: bigint
) => number

/// Result from b2World_RayCastClosest
/// If there is initial overlap the fraction and normal will be zero while the point is an arbitrary point in the overlap region.
/// @ingroup world
export interface b2RayResult
{
	public shapeId: b2ShapeId;
	public point: b2Vec2;
	public normal: b2Vec2;
	public fraction: number;
	public nodeVisits: number;
	public leafVisits: number;
	public hit: boolean;
}

/// World definition used to create a simulation world.
/// Must be initialized using b2DefaultWorldDef().
/// @ingroup world
export interface b2WorldDef
{
	/// Gravity vector. Box2D has no up-vector defined.
	gravity: b2Vec2;

	/// Restitution speed threshold, usually in m/s. Collisions above this
	/// speed have restitution applied (will bounce).
	restitutionThreshold: number;

	/// Threshold speed for hit events. Usually meters per second.
	hitEventThreshold: number;

	/// Contact stiffness. Cycles per second. Increasing this increases the speed of overlap recovery, but can introduce jitter.
	contactHertz: number;

	/// Contact bounciness. Non-dimensional. You can speed up overlap recovery by decreasing this with
	/// the trade-off that overlap resolution becomes more energetic.
	contactDampingRatio: number;

	/// This parameter controls how fast overlap is resolved and usually has units of meters per second. This only
	/// puts a cap on the resolution speed. The resolution speed is increased by increasing the hertz and/or
	/// decreasing the damping ratio.
	contactSpeed: number;

	/// Maximum linear speed. Usually meters per second.
	maximumLinearSpeed: number;

	/// Optional mixing callback for friction. The default uses sqrt(frictionA * frictionB).
	frictionCallback?: b2FrictionCallback;

	/// Optional mixing callback for restitution. The default uses max(restitutionA, restitutionB).
	restitutionCallback?: b2RestitutionCallback;

	/// Can bodies go to sleep to improve performance
	enableSleep: boolean;

	/// Enable continuous collision
	enableContinuous: boolean;

	/// Contact softening when mass ratios are large. Experimental.
	enableContactSoftening?: boolean;

	/// Number of workers to use with the provided task system. Box2D performs best when using only
	/// performance cores and accessing a single L2 cache. Efficiency cores and hyper-threading provide
	/// little benefit and may even harm performance.
	/// @note Box2D does not create threads. This is the number of threads your applications has created
	/// that you are allocating to b2World_Step.
	/// @warning Do not modify the default value unless you are also providing a task system and providing
	/// task callbacks (enqueueTask and finishTask).
	workerCount?: number;

	/// Function to spawn tasks
	enqueueTask?: b2EnqueueTaskCallback;

	/// Function to finish a task
	finishTask?: b2FinishTaskCallback;

	/// User context that is provided to enqueueTask and finishTask
	userTaskContext?: void;

	/// User data
	userData?: void;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: number;
}

/// Use this to initialize your world definition
/// @ingroup world
export const b2DefaultWorldDef: b2WorldDef = 
{
	gravity: new b2Vec2(0, -10),
	hitEventThreshold: b2_lengthUnitsPerMeter.value,
	restitutionThreshold: b2_lengthUnitsPerMeter.value,
	contactSpeed: 3 * b2_lengthUnitsPerMeter.value,
	contactHertz: 30,
	contactDampingRatio: 10,
	maximumLinearSpeed: 400 * b2_lengthUnitsPerMeter.value,
	enableSleep: true,
	enableContinuous: true,
	internalValue: B2_SECRET_COOKIE
}

/// The body simulation type.
/// Each body is one of these three types. The type determines how the body behaves in the simulation.
/// @ingroup body
export enum b2BodyType
{
	/// zero mass, zero velocity, may be manually moved
	b2_staticBody = 0,

	/// zero mass, velocity set by user, moved by solver
	b2_kinematicBody = 1,

	/// positive mass, velocity determined by forces, moved by solver
	b2_dynamicBody = 2,

	/// number of body types
	b2_bodyTypeCount,
}

/// Motion locks to restrict the body movement
export interface b2MotionLocks
{
	/// Prevent translation along the x-axis
	linearX: boolean;

	/// Prevent translation along the y-axis
	linearY: boolean;

	/// Prevent rotation around the z-axis
	angularZ: boolean;
}

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
/// Body definitions are temporary objects used to bundle creation parameters.
/// Must be initialized using b2DefaultBodyDef().
/// @ingroup body
export interface b2BodyDef
{
	/// The body type: static, kinematic, or dynamic.
	type: b2BodyType;

	/// The initial world position of the body. Bodies should be created with the desired position.
	/// @note Creating bodies at the origin and then moving them nearly doubles the cost of body creation, especially
	/// if the body is moved after shapes have been added.
	position?: b2Vec2;

	/// The initial world rotation of the body. Use b2MakeRot() if you have an angle.
	rotation: b2Rot;

	/// The initial linear velocity of the body's origin. Usually in meters per second.
	linearVelocity?: b2Vec2;

	/// The initial angular velocity of the body. Radians per second.
	angularVelocity?: number;

	/// Linear damping is used to reduce the linear velocity. The damping parameter
	/// can be larger than 1 but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	/// Generally linear damping is undesirable because it makes objects move slowly
	/// as if they are floating.
	linearDamping?: number;

	/// Angular damping is used to reduce the angular velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	/// Angular damping can be use slow down rotating bodies.
	angularDamping?: number;

	/// Scale the gravity applied to this body. Non-dimensional.
	gravityScale: number;

	/// Sleep speed threshold, default is 0.05 meters per second
	sleepThreshold: number;

	/// Optional body name for debugging. Up to 31 characters (excluding null termination)
	name?: string;

	/// Use this to store application specific body data.
	userData?: void;

	/// Motions locks to restrict linear and angular movement.
	/// Caution: may lead to softer constraints along the locked direction
	motionLocks?: b2MotionLocks;

	/// Set this flag to false if this body should never fall asleep.
	enableSleep: boolean;

	/// Is this body initially awake or sleeping?
	isAwake: boolean;

	/// Treat this body as high speed object that performs continuous collision detection
	/// against dynamic and kinematic bodies, but not other bullet bodies.
	/// @warning Bullets should be used sparingly. They are not a solution for general dynamic-versus-dynamic
	/// continuous collision.
	isBullet?: boolean;

	/// Used to disable a body. A disabled body does not move or collide.
	isEnabled: boolean;

	/// This allows this body to bypass rotational speed limits. Should only be used
	/// for circular objects, like wheels.
	allowFastRotation?: boolean;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: number;
}

/// Use this to initialize your body definition
/// @ingroup body
export const b2DefaultBodyDef: b2BodyDef = 
{
	type: b2BodyType.b2_staticBody,
	rotation: b2Rot_identity,
	sleepThreshold: 0.05 * b2_lengthUnitsPerMeter.value,
	gravityScale: 1,
	enableSleep: true,
	isAwake: true,
	isEnabled: true,
	internalValue: B2_SECRET_COOKIE,
}

/// This is used to filter collision on shapes. It affects shape-vs-shape collision
/// and shape-versus-query collision (such as b2World_CastRay).
/// @ingroup shape
export interface b2Filter
{
	/// The collision category bits. Normally you would just set one bit. The category bits should
	/// represent your application object types. For example:
	/// @code{.cpp}
	/// enum MyCategories
	/// {
	///    Static  = 0x00000001,
	///    Dynamic = 0x00000002,
	///    Debris  = 0x00000004,
	///    Player  = 0x00000008,
	///    // etc
	/// };
	/// @endcode
	categoryBits: bigint;

	/// The collision mask bits. This states the categories that this
	/// shape would accept for collision.
	/// For example, you may want your player to only collide with static objects
	/// and other players.
	/// @code{.c}
	/// maskBits = Static | Player;
	/// @endcode
	maskBits: bigint;

	/// Collision groups allow a certain group of objects to never collide (negative)
	/// or always collide (positive). A group index of zero has no effect. Non-zero group filtering
	/// always wins against the mask bits.
	/// For example, you may want ragdolls to collide with other ragdolls but you don't want
	/// ragdoll self-collision. In this case you would give each ragdoll a unique negative group index
	/// and apply that group index to all shapes on the ragdoll.
	groupIndex: number;
}

/// Use this to initialize your filter
/// @ingroup shape
export const b2DefaultFilter: b2Filter = 
{
	categoryBits: B2_DEFAULT_CATEGORY_BITS,
	maskBits: B2_DEFAULT_MASK_BITS,
	groupIndex: 0
}

/// The query filter is used to filter collisions between queries and shapes. For example,
/// you may want a ray-cast representing a projectile to hit players and the static environment
/// but not debris.
/// @ingroup shape
export interface b2QueryFilter
{
	/// The collision category bits of this query. Normally you would just set one bit.
	categoryBits: bigint;

	/// The collision mask bits. This states the shape categories that this
	/// query would accept for collision.
	maskBits: bigint;
}

/// Use this to initialize your query filter
/// @ingroup shape
export const b2DefaultQueryFilter: b2QueryFilter =
{
	categoryBits: B2_DEFAULT_CATEGORY_BITS,
	maskBits: B2_DEFAULT_MASK_BITS
}

/// Shape type
/// @ingroup shape
export enum b2ShapeType
{
	/// A circle with an offset
	b2_circleShape,

	/// A capsule is an extruded circle
	b2_capsuleShape,

	/// A line segment
	b2_segmentShape,

	/// A convex polygon
	b2_polygonShape,

	/// A line segment owned by a chain shape
	b2_chainSegmentShape,

	/// The number of shape types
	b2_shapeTypeCount
}

/// Surface materials allow chain shapes to have per segment surface properties.
/// @ingroup shape
export interface b2SurfaceMaterial
{
	/// The Coulomb (dry) friction coefficient, usually in the range [0,1].
	friction: number;

	/// The coefficient of restitution (bounce) usually in the range [0,1].
	/// https://en.wikipedia.org/wiki/Coefficient_of_restitution
	restitution?: number;

	/// The rolling resistance usually in the range [0,1].
	rollingResistance?: number;

	/// The tangent speed for conveyor belts
	tangentSpeed?: number;

	/// User material identifier. This is passed with query results and to friction and restitution
	/// combining functions. It is not used internally.
	userMaterialId?: bigint;

	/// Custom debug draw color.
	customColor?: number;
}

/// Use this to initialize your surface material
/// @ingroup shape
export const b2DefaultSurfaceMaterial: b2SurfaceMaterial =
{
	friction: 0.6
}

/// Used to create a shape.
/// This is a temporary object used to bundle shape creation parameters. You may use
/// the same shape definition to create multiple shapes.
/// Must be initialized using b2DefaultShapeDef().
/// @ingroup shape
export interface b2ShapeDef
{
	/// Use this to store application specific shape data.
	userData?: void;

	/// The surface material for this shape.
	material: b2SurfaceMaterial;

	/// The density, usually in kg/m^2.
	/// This is not part of the surface material because this is for the interior, which may have
	/// other considerations, such as being hollow. For example a wood barrel may be hollow or full of water.
	density: number;

	/// Collision filtering data.
	filter: b2Filter;

	/// Enable custom filtering. Only one of the two shapes needs to enable custom filtering. See b2WorldDef.
	enableCustomFiltering?: boolean;

	/// A sensor shape generates overlap events but never generates a collision response.
	/// Sensors do not have continuous collision. Instead, use a ray or shape cast for those scenarios.
	/// Sensors still contribute to the body mass if they have non-zero density.
	/// @note Sensor events are disabled by default.
	/// @see enableSensorEvents
	isSensor?: boolean;

	/// Enable sensor events for this shape. This applies to sensors and non-sensors. False by default, even for sensors.
	enableSensorEvents?: boolean;

	/// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors. False by default.
	enableContactEvents?: boolean;

	/// Enable hit events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors. False by default.
	enableHitEvents?: boolean;

	/// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
	/// and must be carefully handled due to multithreading. Ignored for sensors.
	enablePreSolveEvents?: boolean;

	/// When shapes are created they will scan the environment for collision the next time step. This can significantly slow down
	/// static body creation when there are many static shapes.
	/// This is flag is ignored for dynamic and kinematic shapes which always invoke contact creation.
	invokeContactCreation: boolean;

	/// Should the body update the mass properties when this shape is created. Default is true.
	updateBodyMass: boolean;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: number;
}

/// Use this to initialize your shape definition
/// @ingroup shape
export const b2DefaultShapeDef: b2ShapeDef =
{
	material: b2DefaultSurfaceMaterial,
	density: 1,
	filter: b2DefaultFilter,
	updateBodyMass: true,
	invokeContactCreation: true,
	internalValue: B2_SECRET_COOKIE
}

/// Used to create a chain of line segments. This is designed to eliminate ghost collisions with some limitations.
/// - chains are one-sided
/// - chains have no mass and should be used on static bodies
/// - chains have a counter-clockwise winding order (normal points right of segment direction)
/// - chains are either a loop or open
/// - a chain must have at least 4 points
/// - the distance between any two points must be greater than B2_LINEAR_SLOP
/// - a chain shape should not self intersect (this is not validated)
/// - an open chain shape has NO COLLISION on the first and final edge
/// - you may overlap two open chains on their first three and/or last three points to get smooth collision
/// - a chain shape creates multiple line segment shapes on the body
/// https://en.wikipedia.org/wiki/Polygonal_chain
/// Must be initialized using b2DefaultChainDef().
/// @warning Do not use chain shapes unless you understand the limitations. This is an advanced feature.
/// @ingroup shape
export interface b2ChainDef
{
	/// Use this to store application specific shape data.
	userData?: void;

	/// An array of at least 4 points. These are cloned and may be temporary.
	points?: b2Vec2;

	/// The point count, must be 4 or more.
	count?: number;

	/// Surface materials for each segment. These are cloned.
	materials: b2SurfaceMaterial;

	/// The material count. Must be 1 or count. This allows you to provide one
	/// material for all segments or a unique material per segment.
	materialCount: number;

	/// Contact filtering data.
	filter: b2Filter;

	/// Indicates a closed chain formed by connecting the first and last points
	isLoop?: boolean;

	/// Enable sensors to detect this chain. False by default.
	enableSensorEvents?: boolean;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: number;
}

/// Use this to initialize your chain definition
/// @ingroup shape
export const b2DefaultChainDef: b2ChainDef =
{
	materials: b2DefaultSurfaceMaterial,
	materialCount: 1,
	filter: b2DefaultFilter,
	internalValue: B2_SECRET_COOKIE
}

//! @cond
/// Profiling data. Times are in milliseconds.
export interface b2Profile
{
	step: number;
	pairs: number;
	collide: number;
	solve: number;
	prepareStages: number;
	solveConstraints: number;
	prepareConstraints: number;
	integrateVelocities: number;
	warmStart: number;
	solveImpulses: number;
	integratePositions: number;
	relaxImpulses: number;
	applyRestitution: number;
	storeImpulses: number;
	splitIslands: number;
	transforms: number;
	sensorHits: number;
	jointEvents: number;
	hitEvents: number;
	refit: number;
	bullets: number;
	sleepIslands: number;
	sensors: number;
}

/// Counters that give details of the simulation size.
export interface b2Counters
{
	bodyCount: number;
	shapeCount: number;
	contactCount: number;
	jointCount: number;
	islandCount: number;
	stackUsed: number;
	staticTreeHeight: number;
	treeHeight: number;
	byteCount: number;
	taskCount: number;
	colorCounts: number[];
}

/// Joint type enumeration
///
/// This is useful because all joint types use b2JointId and sometimes you
/// want to get the type of a joint.
/// @ingroup joint
export enum b2JointType
{
	b2_distanceJoint,
	b2_filterJoint,
	b2_motorJoint,
	b2_prismaticJoint,
	b2_revoluteJoint,
	b2_weldJoint,
	b2_wheelJoint,
}

/// Base joint definition used by all joint types.
/// The local frames are measured from the body's origin rather than the center of mass because:
/// 1. you might not know where the center of mass will be
/// 2. if you add/remove shapes from a body and recompute the mass, the joints will be broken
export interface b2JointDef
{
	/// User data pointer
	userData: void;

	/// The first attached body
	bodyIdA: b2BodyId;

	/// The second attached body
	bodyIdB: b2BodyId;

	/// The first local joint frame
	localFrameA: b2Transform;

	/// The second local joint frame
	localFrameB: b2Transform;

	/// Force threshold for joint events
	forceThreshold: number;

	/// Torque threshold for joint events
	torqueThreshold: number;

	/// Constraint hertz (advanced feature)
	constraintHertz: number;

	/// Constraint damping ratio (advanced feature)
	constraintDampingRatio: number;

	/// Debug draw scale
	drawScale: number;

	/// Set this flag to true if the attached bodies should collide
	collideConnected: boolean;
}

/// Distance joint definition
/// Connects a point on body A with a point on body B by a segment.
/// Useful for ropes and springs.
/// @ingroup distance_joint
export interface b2DistanceJointDef
{
	/// Base joint definition
	base: b2JointDef;

	/// The rest length of this joint. Clamped to a stable minimum value.
	length: number;

	/// Enable the distance constraint to behave like a spring. If false
	/// then the distance joint will be rigid, overriding the limit and motor.
	enableSpring: number;

	/// The lower spring force controls how much tension it can sustain
	lowerSpringForce: number;

	/// The upper spring force controls how much compression it an sustain
	upperSpringForce: number;

	/// The spring linear stiffness Hertz, cycles per second
	hertz: number;

	/// The spring linear damping ratio, non-dimensional
	dampingRatio: number;

	/// Enable/disable the joint limit
	enableLimit: boolean;

	/// Minimum length. Clamped to a stable minimum value.
	minLength: number;

	/// Maximum length. Must be greater than or equal to the minimum length.
	maxLength: number;

	/// Enable/disable the joint motor
	enableMotor: boolean;

	/// The maximum motor force, usually in newtons
	maxMotorForce: number;

	/// The desired motor speed, usually in meters per second
	motorSpeed: number;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: number;
}

/// A motor joint is used to control the relative velocity and or transform between two bodies.
/// With a velocity of zero this acts like top-down friction.
/// @ingroup motor_joint
export interface b2MotorJointDef
{
	/// Base joint definition
	base: b2JointDef;

	/// The desired linear velocity
	linearVelocity: b2Vec2;

	/// The maximum motor force in newtons
	maxVelocityForce: number;

	/// The desired angular velocity
	angularVelocity: number;

	/// The maximum motor torque in newton-meters
	maxVelocityTorque: number;

	/// Linear spring hertz for position control
	linearHertz: number;

	/// Linear spring damping ratio
	linearDampingRatio: number;

	/// Maximum spring force in newtons
	maxSpringForce: number;

	/// Angular spring hertz for position control
	angularHertz: number;

	/// Angular spring damping ratio
	angularDampingRatio: number;

	/// Maximum spring torque in newton-meters
	maxSpringTorque: number;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: number;
}

/// A filter joint is used to disable collision between two specific bodies.
///
/// @ingroup filter_joint
export interface b2FilterJointDef
{
	/// Base joint definition
	base: b2JointDef;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: number;
}

/// Prismatic joint definition
/// Body B may slide along the x-axis in local frame A. Body B cannot rotate relative to body A.
/// The joint translation is zero when the local frame origins coincide in world space.
/// @ingroup prismatic_joint
export interface b2PrismaticJointDef
{
	/// Base joint definition
	base: b2JointDef;

	/// Enable a linear spring along the prismatic joint axis
	enableSpring: boolean;

	/// The spring stiffness Hertz, cycles per second
	hertz: number;

	/// The spring damping ratio, non-dimensional
	dampingRatio: number;

	/// The target translation for the joint in meters. The spring-damper will drive
	/// to this translation.
	targetTranslation: number;

	/// Enable/disable the joint limit
	enableLimit: boolean;

	/// The lower translation limit
	lowerTranslation: number;

	/// The upper translation limit
	upperTranslation: number;

	/// Enable/disable the joint motor
	enableMotor: boolean;

	/// The maximum motor force, typically in newtons
	maxMotorForce: number;

	/// The desired motor speed, typically in meters per second
	motorSpeed: number;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: number;
}

/// Revolute joint definition
/// A point on body B is fixed to a point on body A. Allows relative rotation.
/// @ingroup revolute_joint
export interface b2RevoluteJointDef
{
	/// Base joint definition
	base: b2JointDef;

	/// The target angle for the joint in radians. The spring-damper will drive
	/// to this angle.
	targetAngle: number;

	/// Enable a rotational spring on the revolute hinge axis
	enableSpring: boolean;

	/// The spring stiffness Hertz, cycles per second
	hertz: number;

	/// The spring damping ratio, non-dimensional
	dampingRatio: number;

	/// A flag to enable joint limits
	enableLimit: boolean;

	/// The lower angle for the joint limit in radians. Minimum of -0.99*pi radians.
	lowerAngle: number;

	/// The upper angle for the joint limit in radians. Maximum of 0.99*pi radians.
	upperAngle: number;

	/// A flag to enable the joint motor
	enableMotor: boolean;

	/// The maximum motor torque, typically in newton-meters
	maxMotorTorque: number;

	/// The desired motor speed in radians per second
	motorSpeed: number;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: number;
}

/// Weld joint definition
/// Connects two bodies together rigidly. This constraint provides springs to mimic
/// soft-body simulation.
/// @note The approximate solver in Box2D cannot hold many bodies together rigidly
/// @ingroup weld_joint
export interface b2WeldJointDef
{
	/// Base joint definition
	base: b2JointDef;

	/// Linear stiffness expressed as Hertz (cycles per second). Use zero for maximum stiffness.
	linearHertz: number;

	/// Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.
	angularHertz: number;

	/// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	linearDampingRatio: number;

	/// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	angularDampingRatio: number;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: number;
}

/// Wheel joint definition
/// Body B is a wheel that may rotate freely and slide along the local x-axis in frame A.
/// The joint translation is zero when the local frame origins coincide in world space.
/// @ingroup wheel_joint
export interface b2WheelJointDef
{
	/// Base joint definition
	base: b2JointDef;

	/// Enable a linear spring along the local axis
	enableSpring: boolean;

	/// Spring stiffness in Hertz
	hertz: number;

	/// Spring damping ratio, non-dimensional
	dampingRatio: number;

	/// Enable/disable the joint linear limit
	enableLimit: boolean;

	/// The lower translation limit
	lowerTranslation: number;

	/// The upper translation limit
	upperTranslation: number;

	/// Enable/disable the joint rotational motor
	enableMotor: boolean;

	/// The maximum motor torque, typically in newton-meters
	maxMotorTorque: number;

	/// The desired motor speed in radians per second
	motorSpeed: number;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: number;
}

/// The explosion definition is used to configure options for explosions. Explosions
/// consider shape geometry when computing the impulse.
/// @ingroup world
export interface b2ExplosionDef
{
	/// Mask bits to filter shapes
	maskBits: bigint;

	/// The center of the explosion in world space
	position: b2Vec2;

	/// The radius of the explosion
	radius: number;

	/// The falloff distance beyond the radius. Impulse is reduced to zero at this distance.
	falloff: number;

	/// Impulse per unit length. This applies an impulse according to the shape perimeter that
	/// is facing the explosion. Explosions only apply to circles, capsules, and polygons. This
	/// may be negative for implosions.
	impulsePerLength: number;
}

/**
 * @defgroup events Events
 * World event types.
 *
 * Events are used to collect events that occur during the world time step. These events
 * are then available to query after the time step is complete. This is preferable to callbacks
 * because Box2D uses multithreaded simulation.
 *
 * Also when events occur in the simulation step it may be problematic to modify the world, which is
 * often what applications want to do when events occur.
 *
 * With event arrays, you can scan the events in a loop and modify the world. However, you need to be careful
 * that some event data may become invalid. There are several samples that show how to do this safely.
 *
 * @{
 */

/// A begin touch event is generated when a shape starts to overlap a sensor shape.
export interface b2SensorBeginTouchEvent
{
	/// The id of the sensor shape
	sensorShapeId: b2ShapeId;

	/// The id of the shape that began touching the sensor shape
	visitorShapeId: b2ShapeId;
}

/// An end touch event is generated when a shape stops overlapping a sensor shape.
///	These include things like setting the transform, destroying a body or shape, or changing
///	a filter. You will also get an end event if the sensor or visitor are destroyed.
///	Therefore you should always confirm the shape id is valid using b2Shape_IsValid.
export interface b2SensorEndTouchEvent
{
	/// The id of the sensor shape
	///	@warning this shape may have been destroyed
	///	@see b2Shape_IsValid
	sensorShapeId: b2ShapeId;

	/// The id of the shape that stopped touching the sensor shape
	///	@warning this shape may have been destroyed
	///	@see b2Shape_IsValid
	visitorShapeId: b2ShapeId;
}

/// Sensor events are buffered in the world and are available
/// as begin/end overlap event arrays after the time step is complete.
/// Note: these may become invalid if bodies and/or shapes are destroyed
export interface b2SensorEvents
{
	/// Array of sensor begin touch events
	beginEvents: b2SensorBeginTouchEvent;

	/// Array of sensor end touch events
	endEvents: b2SensorEndTouchEvent;

	/// The number of begin touch events
	beginCount: number;

	/// The number of end touch events
	endCount: number;
}

/// A begin touch event is generated when two shapes begin touching.
export interface b2ContactBeginTouchEvent
{
	/// Id of the first shape
	shapeIdA: b2ShapeId;

	/// Id of the second shape
	shapeIdB: b2ShapeId;

	/// The transient contact id. This contact maybe destroyed automatically when the world is modified or simulated.
	/// Used b2Contact_IsValid before using this id.
	contactId: b2ContactId;
}

/// An end touch event is generated when two shapes stop touching.
///	You will get an end event if you do anything that destroys contacts previous to the last
///	world step. These include things like setting the transform, destroying a body
///	or shape, or changing a filter or body type.
export interface b2ContactEndTouchEvent
{
	/// Id of the first shape
	///	@warning this shape may have been destroyed
	///	@see b2Shape_IsValid
	shapeIdA: b2ShapeId;

	/// Id of the second shape
	///	@warning this shape may have been destroyed
	///	@see b2Shape_IsValid
	shapeIdB: b2ShapeId;

	/// Id of the contact.
	///	@warning this contact may have been destroyed
	///	@see b2Contact_IsValid
	contactId: b2ContactId;
}

/// A hit touch event is generated when two shapes collide with a speed faster than the hit speed threshold.
/// This may be reported for speculative contacts that have a confirmed impulse.
export interface b2ContactHitEvent
{
	/// Id of the first shape
	shapeIdA: b2ShapeId;

	/// Id of the second shape
	shapeIdB: b2ShapeId;

	/// Point where the shapes hit at the beginning of the time step.
	/// This is a mid-point between the two surfaces. It could be at speculative
	/// point where the two shapes were not touching at the beginning of the time step.
	point: b2Vec2;

	/// Normal vector pointing from shape A to shape B
	normal: b2Vec2;

	/// The speed the shapes are approaching. Always positive. Typically in meters per second.
	approachSpeed: number;
}

/// Contact events are buffered in the Box2D world and are available
/// as event arrays after the time step is complete.
/// Note: these may become invalid if bodies and/or shapes are destroyed
export interface b2ContactEvents
{
	/// Array of begin touch events
	beginEvents: b2ContactBeginTouchEvent;

	/// Array of end touch events
	endEvents: b2ContactEndTouchEvent;

	/// Array of hit events
	hitEvents: b2ContactHitEvent;

	/// Number of begin touch events
	beginCount: number;

	/// Number of end touch events
	endCount: number;

	/// Number of hit events
	hitCount: number;
}

/// Body move events triggered when a body moves.
/// Triggered when a body moves due to simulation. Not reported for bodies moved by the user.
/// This also has a flag to indicate that the body went to sleep so the application can also
/// sleep that actor/entity/object associated with the body.
/// On the other hand if the flag does not indicate the body went to sleep then the application
/// can treat the actor/entity/object associated with the body as awake.
/// This is an efficient way for an application to update game object transforms rather than
/// calling functions such as b2Body_GetTransform() because this data is delivered as a contiguous array
/// and it is only populated with bodies that have moved.
/// @note If sleeping is disabled all dynamic and kinematic bodies will trigger move events.
export interface b2BodyMoveEvent
{
	userData: void;
	transform: b2Transform;
	bodyId: b2BodyId;
	fellAsleep: boolean;
}

/// Body events are buffered in the Box2D world and are available
/// as event arrays after the time step is complete.
/// Note: this data becomes invalid if bodies are destroyed
export interface b2BodyEvents
{
	/// Array of move events
	moveEvents: b2BodyMoveEvent;

	/// Number of move events
	moveCount: number;
}

/// Joint events report joints that are awake and have a force and/or torque exceeding the threshold
/// The observed forces and torques are not returned for efficiency reasons.
export interface b2JointEvent
{
	/// The joint id
	jointId: b2JointId;

	/// The user data from the joint for convenience
	userData: void;
}

/// Joint events are buffered in the world and are available
/// as event arrays after the time step is complete.
/// Note: this data becomes invalid if joints are destroyed
export interface b2JointEvents
{
	/// Array of events
	jointEvents: b2JointEvent;

	/// Number of events
	count: number;
}

/// The contact data for two shapes. By convention the manifold normal points
/// from shape A to shape B.
/// @see b2Shape_GetContactData() and b2Body_GetContactData()
export interface b2ContactData
{
	contactId: b2ContactId;
	shapeIdA: b2ShapeId;
	shapeIdB: b2ShapeId;
	manifold: b2Manifold;
}

/// Prototype for a contact filter callback.
/// This is called when a contact pair is considered for collision. This allows you to
/// perform custom logic to prevent collision between shapes. This is only called if
/// one of the two shapes has custom filtering enabled.
/// Notes:
/// - this function must be thread-safe
/// - this is only called if one of the two shapes has enabled custom filtering
/// - this may be called for awake dynamic bodies and sensors
/// Return false if you want to disable the collision
/// @see b2ShapeDef
/// @warning Do not attempt to modify the world inside this callback
/// @ingroup world
export type b2CustomFilterFcn = (
	shapeIdA: b2ShapeId,
	shapeIdB: b2ShapeId,
	context: void
) => boolean;

/// Prototype for a pre-solve callback.
/// This is called after a contact is updated. This allows you to inspect a
/// contact before it goes to the solver. If you are careful, you can modify the
/// contact manifold (e.g. modify the normal).
/// Notes:
/// - this function must be thread-safe
/// - this is only called if the shape has enabled pre-solve events
/// - this is called only for awake dynamic bodies
/// - this is not called for sensors
/// - the supplied manifold has impulse values from the previous step
/// Return false if you want to disable the contact this step
/// @warning Do not attempt to modify the world inside this callback
/// @ingroup world
export type b2PreSolveFcn = (
	shapeIdA: b2ShapeId,
	shapeIdB: b2ShapeId,
	point: b2Vec2,
	normal: b2Vec2,
	context: void
) => boolean;

/// Prototype callback for overlap queries.
/// Called for each shape found in the query.
/// @see b2World_OverlapABB
/// @return false to terminate the query.
/// @ingroup world
export type b2OverlapResultFcn = (
	shapeId: b2ShapeId,
	context: void
) => boolean;

/// Prototype callback for ray and shape casts.
/// Called for each shape found in the query. You control how the ray cast
/// proceeds by returning a float:
/// return -1: ignore this shape and continue
/// return 0: terminate the ray cast
/// return fraction: clip the ray to this point
/// return 1: don't clip the ray and continue
/// A cast with initial overlap will return a zero fraction and a zero normal.
/// @param shapeId the shape hit by the ray
/// @param point the point of initial intersection
/// @param normal the normal vector at the point of intersection, zero for a shape cast with initial overlap
/// @param fraction the fraction along the ray at the point of intersection, zero for a shape cast with initial overlap
/// @param context the user context
/// @return -1 to filter, 0 to terminate, fraction to clip the ray for closest hit, 1 to continue
/// @see b2World_CastRay
/// @ingroup world
export type b2CastResultFcn = (
	shapeId: b2ShapeId,
	point: b2Vec2,
	normal: b2Vec2,
	fraction: number,
	context: void
) => number;

// Used to collect collision planes for character movers.
// Return true to continue gathering planes.
export type b2PlaneResultFcn = (
	shapeId: b2ShapeId,
	plane: b2PlaneResult,
	context: void
) => boolean;

/// These colors are used for debug draw and mostly match the named SVG colors.
/// See https://www.rapidtables.com/web/color/index.html
/// https://johndecember.com/html/spec/colorsvg.html
/// https://upload.wikimedia.org/wikipedia/commons/2/2b/SVG_Recognized_color_keyword_names.svg
export enum b2HexColor
{
	b2_colorAliceBlue = 0xF0F8FF,
	b2_colorAntiqueWhite = 0xFAEBD7,
	b2_colorAqua = 0x00FFFF,
	b2_colorAquamarine = 0x7FFFD4,
	b2_colorAzure = 0xF0FFFF,
	b2_colorBeige = 0xF5F5DC,
	b2_colorBisque = 0xFFE4C4,
	b2_colorBlack = 0x000000,
	b2_colorBlanchedAlmond = 0xFFEBCD,
	b2_colorBlue = 0x0000FF,
	b2_colorBlueViolet = 0x8A2BE2,
	b2_colorBrown = 0xA52A2A,
	b2_colorBurlywood = 0xDEB887,
	b2_colorCadetBlue = 0x5F9EA0,
	b2_colorChartreuse = 0x7FFF00,
	b2_colorChocolate = 0xD2691E,
	b2_colorCoral = 0xFF7F50,
	b2_colorCornflowerBlue = 0x6495ED,
	b2_colorCornsilk = 0xFFF8DC,
	b2_colorCrimson = 0xDC143C,
	b2_colorCyan = 0x00FFFF,
	b2_colorDarkBlue = 0x00008B,
	b2_colorDarkCyan = 0x008B8B,
	b2_colorDarkGoldenRod = 0xB8860B,
	b2_colorDarkGray = 0xA9A9A9,
	b2_colorDarkGreen = 0x006400,
	b2_colorDarkKhaki = 0xBDB76B,
	b2_colorDarkMagenta = 0x8B008B,
	b2_colorDarkOliveGreen = 0x556B2F,
	b2_colorDarkOrange = 0xFF8C00,
	b2_colorDarkOrchid = 0x9932CC,
	b2_colorDarkRed = 0x8B0000,
	b2_colorDarkSalmon = 0xE9967A,
	b2_colorDarkSeaGreen = 0x8FBC8F,
	b2_colorDarkSlateBlue = 0x483D8B,
	b2_colorDarkSlateGray = 0x2F4F4F,
	b2_colorDarkTurquoise = 0x00CED1,
	b2_colorDarkViolet = 0x9400D3,
	b2_colorDeepPink = 0xFF1493,
	b2_colorDeepSkyBlue = 0x00BFFF,
	b2_colorDimGray = 0x696969,
	b2_colorDodgerBlue = 0x1E90FF,
	b2_colorFireBrick = 0xB22222,
	b2_colorFloralWhite = 0xFFFAF0,
	b2_colorForestGreen = 0x228B22,
	b2_colorFuchsia = 0xFF00FF,
	b2_colorGainsboro = 0xDCDCDC,
	b2_colorGhostWhite = 0xF8F8FF,
	b2_colorGold = 0xFFD700,
	b2_colorGoldenRod = 0xDAA520,
	b2_colorGray = 0x808080,
	b2_colorGreen = 0x008000,
	b2_colorGreenYellow = 0xADFF2F,
	b2_colorHoneyDew = 0xF0FFF0,
	b2_colorHotPink = 0xFF69B4,
	b2_colorIndianRed = 0xCD5C5C,
	b2_colorIndigo = 0x4B0082,
	b2_colorIvory = 0xFFFFF0,
	b2_colorKhaki = 0xF0E68C,
	b2_colorLavender = 0xE6E6FA,
	b2_colorLavenderBlush = 0xFFF0F5,
	b2_colorLawnGreen = 0x7CFC00,
	b2_colorLemonChiffon = 0xFFFACD,
	b2_colorLightBlue = 0xADD8E6,
	b2_colorLightCoral = 0xF08080,
	b2_colorLightCyan = 0xE0FFFF,
	b2_colorLightGoldenRodYellow = 0xFAFAD2,
	b2_colorLightGray = 0xD3D3D3,
	b2_colorLightGreen = 0x90EE90,
	b2_colorLightPink = 0xFFB6C1,
	b2_colorLightSalmon = 0xFFA07A,
	b2_colorLightSeaGreen = 0x20B2AA,
	b2_colorLightSkyBlue = 0x87CEFA,
	b2_colorLightSlateGray = 0x778899,
	b2_colorLightSteelBlue = 0xB0C4DE,
	b2_colorLightYellow = 0xFFFFE0,
	b2_colorLime = 0x00FF00,
	b2_colorLimeGreen = 0x32CD32,
	b2_colorLinen = 0xFAF0E6,
	b2_colorMagenta = 0xFF00FF,
	b2_colorMaroon = 0x800000,
	b2_colorMediumAquaMarine = 0x66CDAA,
	b2_colorMediumBlue = 0x0000CD,
	b2_colorMediumOrchid = 0xBA55D3,
	b2_colorMediumPurple = 0x9370DB,
	b2_colorMediumSeaGreen = 0x3CB371,
	b2_colorMediumSlateBlue = 0x7B68EE,
	b2_colorMediumSpringGreen = 0x00FA9A,
	b2_colorMediumTurquoise = 0x48D1CC,
	b2_colorMediumVioletRed = 0xC71585,
	b2_colorMidnightBlue = 0x191970,
	b2_colorMintCream = 0xF5FFFA,
	b2_colorMistyRose = 0xFFE4E1,
	b2_colorMoccasin = 0xFFE4B5,
	b2_colorNavajoWhite = 0xFFDEAD,
	b2_colorNavy = 0x000080,
	b2_colorOldLace = 0xFDF5E6,
	b2_colorOlive = 0x808000,
	b2_colorOliveDrab = 0x6B8E23,
	b2_colorOrange = 0xFFA500,
	b2_colorOrangeRed = 0xFF4500,
	b2_colorOrchid = 0xDA70D6,
	b2_colorPaleGoldenRod = 0xEEE8AA,
	b2_colorPaleGreen = 0x98FB98,
	b2_colorPaleTurquoise = 0xAFEEEE,
	b2_colorPaleVioletRed = 0xDB7093,
	b2_colorPapayaWhip = 0xFFEFD5,
	b2_colorPeachPuff = 0xFFDAB9,
	b2_colorPeru = 0xCD853F,
	b2_colorPink = 0xFFC0CB,
	b2_colorPlum = 0xDDA0DD,
	b2_colorPowderBlue = 0xB0E0E6,
	b2_colorPurple = 0x800080,
	b2_colorRebeccaPurple = 0x663399,
	b2_colorRed = 0xFF0000,
	b2_colorRosyBrown = 0xBC8F8F,
	b2_colorRoyalBlue = 0x4169E1,
	b2_colorSaddleBrown = 0x8B4513,
	b2_colorSalmon = 0xFA8072,
	b2_colorSandyBrown = 0xF4A460,
	b2_colorSeaGreen = 0x2E8B57,
	b2_colorSeaShell = 0xFFF5EE,
	b2_colorSienna = 0xA0522D,
	b2_colorSilver = 0xC0C0C0,
	b2_colorSkyBlue = 0x87CEEB,
	b2_colorSlateBlue = 0x6A5ACD,
	b2_colorSlateGray = 0x708090,
	b2_colorSnow = 0xFFFAFA,
	b2_colorSpringGreen = 0x00FF7F,
	b2_colorSteelBlue = 0x4682B4,
	b2_colorTan = 0xD2B48C,
	b2_colorTeal = 0x008080,
	b2_colorThistle = 0xD8BFD8,
	b2_colorTomato = 0xFF6347,
	b2_colorTurquoise = 0x40E0D0,
	b2_colorViolet = 0xEE82EE,
	b2_colorWheat = 0xF5DEB3,
	b2_colorWhite = 0xFFFFFF,
	b2_colorWhiteSmoke = 0xF5F5F5,
	b2_colorYellow = 0xFFFF00,
	b2_colorYellowGreen = 0x9ACD32,

	b2_colorBox2DRed = 0xDC3132,
	b2_colorBox2DBlue = 0x30AEBF,
	b2_colorBox2DGreen = 0x8CC924,
	b2_colorBox2DYellow = 0xFFEE8C
}

/// This struct holds callbacks you can implement to draw a Box2D world.
/// This structure should be zero initialized.
/// @ingroup world
typedef struct b2DebugDraw
{
	/// Draw a closed polygon provided in CCW order.
	void ( *DrawPolygonFcn )( const b2Vec2* vertices, int vertexCount, b2HexColor color, void* context );

	/// Draw a solid closed polygon provided in CCW order.
	void ( *DrawSolidPolygonFcn )( b2Transform transform, const b2Vec2* vertices, int vertexCount, float radius, b2HexColor color,
								void* context );

	/// Draw a circle.
	void ( *DrawCircleFcn )( b2Vec2 center, float radius, b2HexColor color, void* context );

	/// Draw a solid circle.
	void ( *DrawSolidCircleFcn )( b2Transform transform, float radius, b2HexColor color, void* context );

	/// Draw a solid capsule.
	void ( *DrawSolidCapsuleFcn )( b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color, void* context );

	/// Draw a line segment.
	void ( *DrawSegmentFcn )( b2Vec2 p1, b2Vec2 p2, b2HexColor color, void* context );

	/// Draw a transform. Choose your own length scale.
	void ( *DrawTransformFcn )( b2Transform transform, void* context );

	/// Draw a point.
	void ( *DrawPointFcn )( b2Vec2 p, float size, b2HexColor color, void* context );

	/// Draw a string in world space
	void ( *DrawStringFcn )( b2Vec2 p, const char* s, b2HexColor color, void* context );

	/// World bounds to use for debug draw
	b2AABB drawingBounds;

	/// Scale to use when drawing forces
	float forceScale;

	/// Global scaling for joint drawing
	float jointScale;

	/// Option to draw shapes
	bool drawShapes;

	/// Option to draw joints
	bool drawJoints;

	/// Option to draw additional information for joints
	bool drawJointExtras;

	/// Option to draw the bounding boxes for shapes
	bool drawBounds;

	/// Option to draw the mass and center of mass of dynamic bodies
	bool drawMass;

	/// Option to draw body names
	bool drawBodyNames;

	/// Option to draw contact points
	bool drawContacts;

	/// Option to visualize the graph coloring used for contacts and joints
	bool drawGraphColors;

	/// Option to draw contact feature ids
	bool drawContactFeatures;

	/// Option to draw contact normals
	bool drawContactNormals;

	/// Option to draw contact normal forces
	bool drawContactForces;

	/// Option to draw contact friction forces
	bool drawFrictionForces;

	/// Option to draw islands as bounding boxes
	bool drawIslands;

	/// User context that is passed as an argument to drawing callback functions
	void* context;
} b2DebugDraw;

/// Use this to initialize your drawing interface. This allows you to implement a sub-set
/// of the drawing functions.
B2_API b2DebugDraw b2DefaultDebugDraw( void );