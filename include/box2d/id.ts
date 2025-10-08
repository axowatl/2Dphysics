/// World id references a world instance. This should be treated as an opaque handle.
export class b2WorldId
{
	public index1: number;
	public generation: number;

    constructor(index1: number, generation: number) {
        this.index1 = index1;
        this.generation = generation;
    }
}

/// Body id references a body instance. This should be treated as an opaque handle.
export class b2BodyId
{
	public index1: number;
	public world0: number;
	public generation: number;

    constructor(index1: number, world0: number, generation: number) {
        this.index1 = index1;
        this.world0 = world0;
        this.generation = generation;
    }
}

/// Shape id references a shape instance. This should be treated as an opaque handle.
export class b2ShapeId
{
	public index1: number;
	public world0: number;
	public generation: number;

    constructor(index1: number, world0: number, generation: number) {
        this.index1 = index1;
        this.world0 = world0;
        this.generation = generation;
    }
}

/// Chain id references a chain instances. This should be treated as an opaque handle.
export class b2ChainId
{
	public index1: number;
	public world0: number;
	public generation: number;

    constructor(index1: number, world0: number, generation: number) {
        this.index1 = index1;
        this.world0 = world0;
        this.generation = generation;
    }
}

/// Joint id references a joint instance. This should be treated as an opaque handle.
export class b2JointId
{
	public index1: number;
	public world0: number;
	public generation: number;

    constructor(index1: number, world0: number, generation: number) {
        this.index1 = index1;
        this.world0 = world0;
        this.generation = generation;
    }
}

/// Contact id references a contact instance. This should be treated as an opaque handled.
export class b2ContactId
{
	public index1: number;
	public world0: number;
	public padding: number;
	public generation: number;

    constructor(index1: number, world0: number, padding: number, generation: number) {
        this.index1 = index1;
        this.world0 = world0;
        this.padding = padding;
        this.generation = generation;
    }
}

/// Use these to make your identifiers null.
/// You may also use zero initialization to get null.
export static const b2_nullWorldId: b2WorldId = new b2WorldId(0, 0);
export static const b2_nullBodyId: b2BodyId = new b2BodyId(0, 0, 0);
export static const b2_nullShapeId: b2ShapeId = new b2ShapeId(0, 0, 0);
export static const b2_nullChainId: b2ChainId = new b2ChainId(0, 0, 0);
export static const b2_nullJointId: b2JointId = new b2JointId(0, 0, 0);
export static const b2_nullContactId: b2ContactId = new b2ContactId(0, 0, 0, 0);

/// Macro to determine if any id is null.
export function B2_IS_NULL(id: b2WorldId | b2BodyId | b2ShapeId | b2ChainId | b2JointId | b2ChainId): boolean
{
    return id.index1 == 0;
}

/// Macro to determine if any id is non-null.
export function B2_IS_NON_NULL(id: b2WorldId | b2BodyId | b2ShapeId | b2ChainId | b2JointId | b2ChainId): boolean
{
    return id.index1 != 0;
}

/// Compare two ids for equality. Doesn't work for b2WorldId. Don't mix types.
export function B2_ID_EQUALS(id1: b2BodyId | b2ShapeId | b2ChainId | b2JointId | b2ChainId, id2: b2BodyId | b2ShapeId | b2ChainId | b2JointId | b2ChainId): boolean
{
    return id1.index1 == id2.index1 && id1.world0 == id2.world0 && id1.generation == id2.generation;
}

/// Store a world id into a uint32_t.
export function b2StoreWorldId(id: b2WorldId): number
{
	return (id.index1 << 16) | id.generation;
}

/// Load a uint32_t into a world id.
export function b2LoadWorldId(x: number): b2WorldId
{
	const id: b2WorldId = new b2WorldId(x >> 16, x);
	return id;
}

/// Store a body id into a uint64_t.
export function b2StoreBodyId(id: b2BodyId): number
{
	return (id.index1 << 32) | (id.world0 << 16) | id.generation;
}

/// Load a uint64_t into a body id.
export function b2LoadBodyId(x: number): b2BodyId
{
	const id: b2BodyId = new b2BodyId(x >> 32, x >> 16, x);
	return id;
}

/// Store a shape id into a uint64_t.
export function b2StoreShapeId(id: b2ShapeId): number
{
	return (id.index1 << 32) | (id.world0 << 16) | id.generation;
}

/// Load a uint64_t into a shape id.
export function b2LoadShapeId(x: number): b2ShapeId
{
	const id: b2ShapeId = new b2ShapeId(x >> 32, x >> 16, x);
	return id;
}

/// Store a chain id into a uint64_t.
export function b2StoreChainId(id: b2ChainId): number
{
	return (id.index1 << 32) | (id.world0 << 16) | id.generation;
}

/// Load a uint64_t into a chain id.
export function b2LoadChainId(x: number): b2ChainId
{
	const id: b2ChainId = new b2ChainId(x >> 32,x >> 16, x);
	return id;
}

/// Store a joint id into a uint64_t.
export function b2StoreJointId(id: b2JointId): number
{
	return (id.index1 << 32) | (id.world0 << 16) | id.generation;
}

/// Load a uint64_t into a joint id.
export function b2LoadJointId(x: number): b2JointId
{
	const id: b2JointId = new b2JointId(x >> 32, x >> 16, x);
	return id;
}

/// Store a contact id into 16 bytes
B2_ID_INLINE void b2StoreContactId( b2ContactId id, uint32_t values[3] )
{
	values[0] = (uint32_t)id.index1;
	values[1] = (uint32_t)id.world0;
	values[2] = (uint32_t)id.generation;
}

/// Load a two uint64_t into a contact id.
B2_ID_INLINE b2ContactId b2LoadContactId( uint32_t values[3] )
{
	b2ContactId id;
	id.index1 = (int32_t)values[0];
	id.world0 = (uint16_t)values[1];
	id.padding = 0;
	id.generation = (uint32_t)values[2];
	return id;
}

/**@}*/