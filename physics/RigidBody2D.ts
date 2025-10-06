/*
Properties
Property	Description
angularDamping	The angular damping of the object.
angularVelocity	The angular velocity vector of the rigidbody measured in radians per second.
automaticCenterOfMass	Whether or not to calculate the center of mass automatically.
automaticInertiaTensor	Whether or not to calculate the inertia tensor automatically.
centerOfMass	The center of mass relative to the transform's origin.
collisionDetectionMode	The Rigidbody's collision detection mode.
constraints	Controls which degrees of freedom are allowed for the simulation of this Rigidbody.
detectCollisions	Should collision detection be enabled? (By default always enabled).
excludeLayers	The additional layers that all Colliders attached to this Rigidbody should exclude when deciding if the Collider can come into contact with another Collider.
freezeRotation	Controls whether physics will change the rotation of the object.
includeLayers	The additional layers that all Colliders attached to this Rigidbody should include when deciding if the Collider can come into contact with another Collider.
inertiaTensor	The inertia tensor of this body, defined as a diagonal matrix in a reference frame positioned at this body's center of mass and rotated by Rigidbody.inertiaTensorRotation.
inertiaTensorRotation	The rotation of the inertia tensor.
interpolation	Interpolation provides a way to manage the appearance of jitter in the movement of your Rigidbody GameObjects at run time.
isKinematic	Controls whether physics affects the rigidbody.
linearDamping	The linear damping of the Rigidbody linear velocity.
linearVelocity	The linear velocity vector of the rigidbody. It represents the rate of change of Rigidbody position.
mass	The mass of the rigidbody.
maxAngularVelocity	The maximum angular velocity of the rigidbody measured in radians per second. (Default 7) range { 0, infinity }.
maxDepenetrationVelocity	Maximum velocity of a rigidbody when moving out of penetrating state.
maxLinearVelocity	The maximum linear velocity of the rigidbody measured in meters per second.
position	The position of the rigidbody.
rotation	The rotation of the Rigidbody.
sleepThreshold	The mass-normalized energy threshold, below which objects start going to sleep.
solverIterations	The solverIterations determines how accurately Rigidbody joints and collision contacts are resolved. Overrides Physics.defaultSolverIterations. Must be positive.
solverVelocityIterations	The solverVelocityIterations affects how how accurately Rigidbody joints and collision contacts are resolved. Overrides Physics.defaultSolverVelocityIterations. Must be positive.
useGravity	Controls whether gravity affects this rigidbody.
worldCenterOfMass	The center of mass of the rigidbody in world space (Read Only).
Public Methods
Method	Description
AddExplosionForce	Applies a force to a rigidbody that simulates explosion effects.
AddForce	Adds a force to the Rigidbody.
AddForceAtPosition	Applies force at position. As a result this will apply a torque and force on the object.
AddRelativeForce	Adds a force to the rigidbody relative to its coordinate system.
AddRelativeTorque	Adds a torque to the rigidbody relative to its coordinate system.
AddTorque	Adds a torque to the rigidbody.
ClosestPointOnBounds	The closest point to the bounding box of the attached colliders.
GetAccumulatedForce	Returns the force that the Rigidbody has accumulated before the simulation step.
GetAccumulatedTorque	Returns the torque that the Rigidbody has accumulated before the simulation step.
GetPointVelocity	The velocity of the rigidbody at the point worldPoint in global space.
GetRelativePointVelocity	The velocity relative to the rigidbody at the point relativePoint.
IsSleeping	Is the rigidbody sleeping?
Move	Moves the Rigidbody to position and rotates the Rigidbody to rotation.
MovePosition	Moves the kinematic Rigidbody towards position.
MoveRotation	Rotates the rigidbody to rotation.
PublishTransform	Applies the position and rotation of the Rigidbody to the corresponding Transform component.
ResetCenterOfMass	Reset the center of mass of the rigidbody.
ResetInertiaTensor	Reset the inertia tensor value and rotation.
Sleep	Forces a rigidbody to sleep until woken up.
SweepTest	Tests if a rigidbody would collide with anything, if it was moved through the Scene.
SweepTestAll	Like Rigidbody.SweepTest, but returns all hits.
WakeUp	Forces a rigidbody to wake up.
Messages
Message	Description
OnCollisionEnter	OnCollisionEnter is called when this collider/rigidbody has begun touching another rigidbody/collider.
OnCollisionExit	OnCollisionExit is called when this collider/rigidbody has stopped touching another rigidbody/collider.
OnCollisionStay	OnCollisionStay is called once per frame for every Collider or Rigidbody that touches another Collider or Rigidbody.
Inherited Members
Properties
Property	Description
gameObject	The game object this component is attached to. A component is always attached to a game object.
tag	The tag of this game object.
transform	The Transform attached to this GameObject.
hideFlags	Should the object be hidden, saved with the Scene or modifiable by the user?
name	The name of the object.
Public Methods
Method	Description
BroadcastMessage	Calls the method named methodName on every MonoBehaviour in this game object or any of its children.
CompareTag	Checks the GameObject's tag against the defined tag.
GetComponent	Gets a reference to a component of type T on the same GameObject as the component specified.
GetComponentInChildren	Gets a reference to a component of type T on the same GameObject as the component specified, or any child of the GameObject.
GetComponentIndex	Gets the index of the component on its parent GameObject.
GetComponentInParent	Gets a reference to a component of type T on the same GameObject as the component specified, or any parent of the GameObject.
GetComponents	Gets references to all components of type T on the same GameObject as the component specified.
GetComponentsInChildren	Gets references to all components of type T on the same GameObject as the component specified, and any child of the GameObject.
GetComponentsInParent	Gets references to all components of type T on the same GameObject as the component specified, and any parent of the GameObject.
SendMessage	Calls the method named methodName on every MonoBehaviour in this game object.
SendMessageUpwards	Calls the method named methodName on every MonoBehaviour in this game object and on every ancestor of the behaviour.
TryGetComponent	Gets the component of the specified type, if it exists.
GetInstanceID	Gets the instance ID of the object.
ToString	Returns the name of the object.
Static Methods
Method	Description
Destroy	Removes a GameObject, component, or asset.
DestroyImmediate	Destroys the specified object immediately. Use with caution and in Edit mode only.
DontDestroyOnLoad	Do not destroy the target Object when loading a new Scene.
FindAnyObjectByType	Retrieves any active loaded object of Type type.
FindFirstObjectByType	Retrieves the first active loaded object of Type type.
FindObjectsByType	Retrieves a list of all loaded objects of Type type.
Instantiate	Clones the object original and returns the clone.
InstantiateAsync	Captures a snapshot of the original object (that must be related to some GameObject) and returns the AsyncInstantiateOperation.
Operators
Operator	Description
bool	Does the object exist?
operator !=	Compares if two objects refer to a different object.
operator ==	Compares two object references to see if they refer to the same object.
*/

class Rigidbody2D {
    private _angularDamping: number;
    private _angularVelocity: Vector3;

    constructor() {
        this._angularDamping = 0.05;
    }

    get angularDamping(): number {
        return this._angularDamping;
    }

    set angularDamping(value: number) {
        if (value < 0 || value > 1) {
            console.error(`Failed to set angularDamping on objectName, value(${value}) is out of bounds 0 - 1.`);
        }
        this._angularDamping = value;
    }


}
