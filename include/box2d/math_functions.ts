//#region Math
/**
 * @defgroup math Math
 * @brief Vector math types and functions
 * @{
 */

/// 2D vector
/// This can be used to represent a point or free vector
class b2Vec2
{
	/// coordinates
	public x: number;
    public y: number;

    constructor(x: number, y: number) {
        this.x = x;
        this.y = y;
    }
}

/// Cosine and sine pair
/// This uses a custom implementation designed for cross-platform determinism
class b2CosSin
{
	/// cosine and sine
	public cosine: number;
	public sine: number;

    constructor(c: number, s: number) {
        this.cosine = c;
        this.sine = s;
    }
}

/// 2D rotation
/// This is similar to using a complex number for rotation
class b2Rot
{
	/// cosine and sine
	public c: number;
    public s: number;

    constructor(c: number, s: number) {
        this.c = c;
        this.s = s;
    }
}

/// A 2D rigid transform
class b2Transform
{
	public p: b2Vec2;
	public q: b2Rot;

    constructor(p: b2Vec2, q: b2Rot) {
        this.p = p;
        this.q = q;
    }
}

/// A 2-by-2 Matrix
class b2Mat22
{
	/// columns
    public cx: b2Vec2;
    public cy: b2Vec2;
	
    constructor(cx: b2Vec2, cy: b2Vec2) {
        this.cx = cx;
        this.cy = cy;
    }
}

/// Axis-aligned bounding box
class b2AABB
{
	public lowerBound: b2Vec2;
	public upperBound: b2Vec2;

    constructor(lb: b2Vec2, ub: b2Vec2) {
        this.lowerBound = lb;
        this.upperBound = ub;
    }
}

/// separation = dot(normal, point) - offset
class b2Plane
{
	public normal: b2Vec2;
	public offset: number;

    constructor(n: b2Vec2, o: number) {
        this.normal = n;
        this.offset = o;
    }
}

/**@}*/
// #endregion Math

/**
 * @addtogroup math
 * @{
 */

/// https://en.wikipedia.org/wiki/Pi
const B2_PI = 3.14159265359;

static const b2Vec2_zero: b2Vec2 = new b2Vec2(0, 0);
static const b2Rot_identity: b2Rot = new b2Rot(1, 0);
static const b2Transform_identity: b2Transform = new b2Transform(new b2Vec2(0, 0), new b2Rot(1, 0));
static const b2Mat22_zero: b2Mat22 = new b2Mat22(new b2Vec2(0, 0), new b2Vec2(0, 0));

/// Is this a valid number? Not NaN or infinity.
function b2IsValidFloat(a: number): boolean
{
	if (isNaN(a))
	{
		return false;
	}

	if (isFinite(a))
	{
		return false;
	}

	return true;
}

/// Is this a valid vector? Not NaN or infinity.
function b2IsValidVec2(v: b2Vec2): boolean
{
	if (isNaN(v.x) || isNaN(v.y))
	{
		return false;
	}

	if (isFinite(v.x) || isFinite(v.y))
	{
		return false;
	}

	return true;
}

/// Is this a valid rotation? Not NaN or infinity. Is normalized.
function b2IsValidRotation(q: b2Rot): boolean
{
	if (isNaN(q.s) || isNaN(q.c))
	{
		return false;
	}

	if (isFinite(q.s) || isFinite(q.c))
	{
		return false;
	}

	return b2IsNormalizedRot(q);
}

/// Is this a valid transform? Not NaN or infinity. Rotation is normalized.
function b2IsValidTransform(t: b2Transform)
{
	if (b2IsValidVec2(t.p) == false)
	{
		return false;
	}

	return b2IsValidRotation(t.q);
}

/// Is this a valid plane? Normal is a unit vector. Not Nan or infinity.
function b2IsValidPlane(a: b2Plane): boolean
{
	return b2IsValidVec2(a.normal) && b2IsNormalized(a.normal) && b2IsValidFloat(a.offset);
}

/// @return a float clamped between a lower and upper bound
function b2ClampFloat(a: number, lower: number, upper: number): number
{
	return a < lower ? lower : ( a > upper ? upper : a );
}

/// Compute an approximate arctangent in the range [-pi, pi]
/// This is hand coded for cross-platform determinism. The atan2f
/// function in the standard library is not cross-platform deterministic.
///	Accurate to around 0.0023 degrees
// https://stackoverflow.com/questions/46210708/atan2-approximation-with-11bits-in-mantissa-on-x86with-sse2-and-armwith-vfpv4
function b2Atan2(y: number, x: number): number
{
	// Added check for (0,0) to match atan2f and avoid NaN
	if (x == 0 && y == 0)
	{
		return 0;
	}

	const ax: number = Math.abs(x);
	const ay: number = Math.abs(y);
	const mx: number = Math.max(ay, ax);
	const mn: number = Math.min(ay, ax);
	const a: number = mn / mx;

	// Minimax polynomial approximation to atan(a) on [0,1]
	const s: number = a * a;
	const c: number = s * a;
	const q: number = s * s;
	let r: number = 0.024840285 * q + 0.18681418;
	const t: number = -0.094097948 * q - 0.33213072;
	r = r * s + t;
	r = r * c + a;

	// Map to full circle
	if (ay > ax)
	{
		r = 1.57079637 - r;
	}

	if (x < 0)
	{
		r = 3.14159274 - r;
	}

	if (y < 0)
	{
		r = -r;
	}

	return r;
}

/// Compute the cosine and sine of an angle in radians. Implemented
/// for cross-platform determinism.
function b2ComputeCosSin(radians: number): b2CosSin
{
	const x: number = b2UnwindAngle(radians);
	const pi2: number = B2_PI * B2_PI;

	// cosine needs angle in [-pi/2, pi/2]
	let c: number;
	if ( x < -0.5 * B2_PI )
	{
		const y: number = x + B2_PI;
		const y2: number = y * y;
		c = -( pi2 - 4 * y2 ) / ( pi2 + y2 );
	}
	else if (x > 0.5 * B2_PI)
	{
		const y: number = x - B2_PI;
		const y2: number = y * y;
		c = -(pi2 - 4 * y2) / (pi2 + y2);
	}
	else
	{
		const y2: number = x * x;
		c = (pi2 - 4 * y2) / (pi2 + y2);
	}

	// sine needs angle in [0, pi]
	let s: number;
	if (x < 0)
	{
		const y: number = x + B2_PI;
		s = -16 * y * (B2_PI - y) / (5 * pi2 - 4 * y * (B2_PI - y));
	}
	else
	{
		s = 16 * x * (B2_PI - x) / (5 * pi2 - 4 * x * (B2_PI - x));
	}

	const mag: number = Math.sqrt(s * s + c * c);
	const invMag: number = mag > 0 ? 1 / mag : 0;
	const cs: b2CosSin = new b2CosSin(c * invMag, s * invMag);
	return cs;
}

/// Vector dot product
function b2Dot(a: b2Vec2, b: b2Vec2): number
{
	return a.x * b.x + a.y * b.y;
}

/// Vector cross product. In 2D this yields a scalar.
function b2Cross(a: b2Vec2, b: b2Vec2): number
{
	return a.x * b.y - a.y * b.x;
}

/// Perform the cross product on a vector and a scalar. In 2D this produces a vector.
function b2CrossVS(v: b2Vec2, s: number): b2Vec2
{
	return new b2Vec2(s * v.y, -s * v.x);
}

/// Perform the cross product on a scalar and a vector. In 2D this produces a vector.
function b2CrossSV(s: number, v: b2Vec2): b2Vec2
{
	return new b2Vec2(-s * v.y, s * v.x);
}

/// Get a left pointing perpendicular vector. Equivalent to b2CrossSV(1.0f, v)
function b2LeftPerp(v: b2Vec2): b2Vec2
{
	return new b2Vec2(-v.y, v.x);
}

/// Get a right pointing perpendicular vector. Equivalent to b2CrossVS(v, 1.0f)
function b2RightPerp(v: b2Vec2): b2Vec2
{
	return new b2Vec2(v.y, -v.x);
}

/// Vector addition
function b2Add(a: b2Vec2, b: b2Vec2): b2Vec2
{
	return new b2Vec2(a.x + b.x, a.y + b.y);
}

/// Vector subtraction
function b2Sub(a: b2Vec2, b: b2Vec2): b2Vec2
{
	return new b2Vec2(a.x - b.x, a.y - b.y);
}

/// Vector negation
function b2Neg(a: b2Vec2): b2Vec2
{
	return new b2Vec2(-a.x, -a.y);
}

/// Vector linear interpolation
/// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
function b2Lerp(a: b2Vec2, b: b2Vec2, t: number): b2Vec2
{
	return new b2Vec2((1 - t) * a.x + t * b.x, (1 - t) * a.y + t * b.y);
}

/// Component-wise multiplication
function b2Mul(a: b2Vec2, b: b2Vec2): b2Vec2
{
	return new b2Vec2(a.x * b.x, a.y * b.y);
}

/// Multiply a scalar and vector
function b2MulSV(s: number, v: b2Vec2): b2Vec2
{
	return new b2Vec2(s * v.x, s * v.y);
}

/// a + s * b
function b2MulAdd(a: b2Vec2, s: number, b: b2Vec2): b2Vec2
{
	return new b2Vec2(a.x + s * b.x, a.y + s * b.y);
}

/// a - s * b
function b2MulSub(a: b2Vec2, s: number, b: b2Vec2): b2Vec2
{
	return new b2Vec2(a.x - s * b.x, a.y - s * b.y);
}

/// Component-wise absolute vector
function b2Abs(a: b2Vec2): b2Vec2
{
	return new b2Vec2(Math.abs(a.x), Math.abs(a.y));
}

/// Component-wise minimum vector
function b2Min(a: b2Vec2, b: b2Vec2): b2Vec2
{
	return new b2Vec2(Math.min(a.x, b.x), Math.min(a.y, b.y));
}

/// Component-wise maximum vector
function b2Max(a: b2Vec2, b: b2Vec2): b2Vec2
{
	return new b2Vec2(Math.max(a.x, b.x), Math.max(a.y, b.y));
}

/// Component-wise clamp vector v into the range [a, b]
function b2Clamp(v: b2Vec2, a: b2Vec2, b: b2Vec2): b2Vec2
{
	let c: b2Vec2 = b2Vec2_zero;
	c.x = b2ClampFloat( v.x, a.x, b.x );
	c.y = b2ClampFloat( v.y, a.y, b.y );
	return c;
}

/// Get the length of this vector (the norm)
function b2Length(v: b2Vec2): number
{
	return Math.sqrt(v.x * v.x + v.y * v.y);
}

/// Get the distance between two points
function b2Distance(a: b2Vec2, b: b2Vec2): number
{
	const dx: number = b.x - a.x;
	const dy: number = b.y - a.y;
	return Math.sqrt( dx * dx + dy * dy );
}

/// Convert a vector into a unit vector if possible, otherwise returns the zero vector.
/// todo MSVC is not inlining this function in several places per warning 4710
function b2Normalize(v: b2Vec2): b2Vec2
{
	const length: number = Math.sqrt( v.x * v.x + v.y * v.y );
	if ( length < Number.EPSILON )
	{
		return b2Vec2_zero;
	}

	const invLength: number = 1 / length;
	const n: b2Vec2 = new b2Vec2(invLength * v.x, invLength * v.y);
	return n;
}

/// Determines if the provided vector is normalized (norm(a) == 1).
function b2IsNormalized(a: b2Vec2): boolean
{
	const aa: number = b2Dot(a, a);
	return Math.abs(1 - aa) < 100 * Number.EPSILON;
}

/// Convert a vector into a unit vector if possible, otherwise returns the zero vector. Also
/// outputs the length.
function b2GetLengthAndNormalize(length: number, v: b2Vec2): b2Vec2
{
	*length = sqrtf( v.x * v.x + v.y * v.y );
	if ( *length < Number.EPSILON )
	{
		return B2_LITERAL( b2Vec2 ){ 0.0f, 0.0f };
	}

	float invLength = 1.0f / *length;
	b2Vec2 n = { invLength * v.x, invLength * v.y };
	return n;
}

/// Normalize rotation
function b2NormalizeRot(q: b2Rot): b2Rot
{
	const mag: number = Math.sqrt( q.s * q.s + q.c * q.c );
	const invMag: number = mag > 0 ? 1 / mag : 0;
	const qn: b2Rot = new b2Rot(q.c * invMag, q.s * invMag);
	return qn;
}

/// Integrate rotation from angular velocity
/// @param q1 initial rotation
/// @param deltaAngle the angular displacement in radians
B2_INLINE b2Rot b2IntegrateRotation( b2Rot q1, float deltaAngle )
{
	// dc/dt = -omega * sin(t)
	// ds/dt = omega * cos(t)
	// c2 = c1 - omega * h * s1
	// s2 = s1 + omega * h * c1
	b2Rot q2 = { q1.c - deltaAngle * q1.s, q1.s + deltaAngle * q1.c };
	float mag = sqrtf( q2.s * q2.s + q2.c * q2.c );
	float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
	b2Rot qn = { q2.c * invMag, q2.s * invMag };
	return qn;
}

/// Get the length squared of this vector
B2_INLINE float b2LengthSquared( b2Vec2 v )
{
	return v.x * v.x + v.y * v.y;
}

/// Get the distance squared between points
B2_INLINE float b2DistanceSquared( b2Vec2 a, b2Vec2 b )
{
	b2Vec2 c = { b.x - a.x, b.y - a.y };
	return c.x * c.x + c.y * c.y;
}

/// Make a rotation using an angle in radians
B2_INLINE b2Rot b2MakeRot( float radians )
{
	b2CosSin cs = b2ComputeCosSin( radians );
	return B2_LITERAL( b2Rot ){ cs.cosine, cs.sine };
}

/// Make a rotation using a unit vector
B2_INLINE b2Rot b2MakeRotFromUnitVector( b2Vec2 unitVector )
{
	B2_ASSERT( b2IsNormalized( unitVector ) );
	return B2_LITERAL( b2Rot ){ unitVector.x, unitVector.y };
}

/// Compute the rotation between two unit vectors
B2_API b2Rot b2ComputeRotationBetweenUnitVectors( b2Vec2 v1, b2Vec2 v2 );

/// Is this rotation normalized?
function b2IsNormalizedRot(q: b2Rot): boolean
{
	// larger tolerance due to failure on mingw 32-bit
	const qq: number = q.s * q.s + q.c * q.c;
	return 1 - 0.0006 < qq && qq < 1 + 0.0006;
}

/// Normalized linear interpolation
/// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
///	https://web.archive.org/web/20170825184056/http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/
B2_INLINE b2Rot b2NLerp( b2Rot q1, b2Rot q2, float t )
{
	float omt = 1.0f - t;
	b2Rot q = {
		omt * q1.c + t * q2.c,
		omt * q1.s + t * q2.s,
	};

	float mag = sqrtf( q.s * q.s + q.c * q.c );
	float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
	b2Rot qn = { q.c * invMag, q.s * invMag };
	return qn;
}

/// Compute the angular velocity necessary to rotate between two rotations over a give time
/// @param q1 initial rotation
/// @param q2 final rotation
/// @param inv_h inverse time step
B2_INLINE float b2ComputeAngularVelocity( b2Rot q1, b2Rot q2, float inv_h )
{
	// ds/dt = omega * cos(t)
	// dc/dt = -omega * sin(t)
	// s2 = s1 + omega * h * c1
	// c2 = c1 - omega * h * s1

	// omega * h * s1 = c1 - c2
	// omega * h * c1 = s2 - s1
	// omega * h = (c1 - c2) * s1 + (s2 - s1) * c1;
	// omega * h = s1 * c1 - c2 * s1 + s2 * c1 - s1 * c1
	// omega * h = s2 * c1 - c2 * s1 = sin(a2 - a1) ~= a2 - a1 for small delta
	float omega = inv_h * ( q2.s * q1.c - q2.c * q1.s );
	return omega;
}

/// Get the angle in radians in the range [-pi, pi]
B2_INLINE float b2Rot_GetAngle( b2Rot q )
{
	return b2Atan2( q.s, q.c );
}

/// Get the x-axis
B2_INLINE b2Vec2 b2Rot_GetXAxis( b2Rot q )
{
	b2Vec2 v = { q.c, q.s };
	return v;
}

/// Get the y-axis
B2_INLINE b2Vec2 b2Rot_GetYAxis( b2Rot q )
{
	b2Vec2 v = { -q.s, q.c };
	return v;
}

/// Multiply two rotations: q * r
B2_INLINE b2Rot b2MulRot( b2Rot q, b2Rot r )
{
	// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
	// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
	// s(q + r) = qs * rc + qc * rs
	// c(q + r) = qc * rc - qs * rs
	b2Rot qr;
	qr.s = q.s * r.c + q.c * r.s;
	qr.c = q.c * r.c - q.s * r.s;
	return qr;
}

/// Transpose multiply two rotations: inv(a) * b
/// This rotates a vector local in frame b into a vector local in frame a
B2_INLINE b2Rot b2InvMulRot( b2Rot a, b2Rot b )
{
	// [ ac as] * [bc -bs] = [ac*bc+qs*bs -ac*bs+as*bc]
	// [-as ac]   [bs  bc]   [-as*bc+ac*bs as*bs+ac*bc]
	// s(a - b) = ac * bs - as * bc
	// c(a - b) = ac * bc + as * bs
	b2Rot r;
	r.s = a.c * b.s - a.s * b.c;
	r.c = a.c * b.c + a.s * b.s;
	return r;
}

/// Relative angle between a and b
B2_INLINE float b2RelativeAngle( b2Rot a, b2Rot b )
{
	// sin(b - a) = bs * ac - bc * as
	// cos(b - a) = bc * ac + bs * as
	float s = a.c * b.s - a.s * b.c;
	float c = a.c *b.c + a.s * b.s;
	return b2Atan2( s, c );
}

/// Convert any angle into the range [-pi, pi]
function b2UnwindAngle(radians: number): number
{
	// Assuming this is deterministic
	return remainderf(radians, 2 * B2_PI);
}

function remainderf(x: number, y: number): number {
  if (y === 0) {
    return NaN;
  }

  const quotient = x / y;

  const roundedQuotient = Math.round(quotient);
  if (Math.abs(quotient - roundedQuotient) === 0.5 && roundedQuotient % 2 !== 0) {
    const roundedQuotientToEven = Math.floor(quotient);
    const quotientDifference = quotient - roundedQuotientToEven;
    const isExactlyHalfwayToEven = (quotientDifference === 0.5);
    const evenQuotient = roundedQuotientToEven;
    const oddQuotient = roundedQuotientToEven + 1;
    const finalQuotient = isExactlyHalfwayToEven ? evenQuotient : oddQuotient;
    return x - finalQuotient * y;
  }

  return x - roundedQuotient * y;
}

/// Rotate a vector
B2_INLINE b2Vec2 b2RotateVector( b2Rot q, b2Vec2 v )
{
	return B2_LITERAL( b2Vec2 ){ q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y };
}

/// Inverse rotate a vector
B2_INLINE b2Vec2 b2InvRotateVector( b2Rot q, b2Vec2 v )
{
	return B2_LITERAL( b2Vec2 ){ q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y };
}

/// Transform a point (e.g. local space to world space)
B2_INLINE b2Vec2 b2TransformPoint( b2Transform t, const b2Vec2 p )
{
	float x = ( t.q.c * p.x - t.q.s * p.y ) + t.p.x;
	float y = ( t.q.s * p.x + t.q.c * p.y ) + t.p.y;

	return B2_LITERAL( b2Vec2 ){ x, y };
}

/// Inverse transform a point (e.g. world space to local space)
B2_INLINE b2Vec2 b2InvTransformPoint( b2Transform t, const b2Vec2 p )
{
	float vx = p.x - t.p.x;
	float vy = p.y - t.p.y;
	return B2_LITERAL( b2Vec2 ){ t.q.c * vx + t.q.s * vy, -t.q.s * vx + t.q.c * vy };
}

/// Multiply two transforms. If the result is applied to a point p local to frame B,
/// the transform would first convert p to a point local to frame A, then into a point
/// in the world frame.
/// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
///    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
B2_INLINE b2Transform b2MulTransforms( b2Transform A, b2Transform B )
{
	b2Transform C;
	C.q = b2MulRot( A.q, B.q );
	C.p = b2Add( b2RotateVector( A.q, B.p ), A.p );
	return C;
}

/// Creates a transform that converts a local point in frame B to a local point in frame A.
/// v2 = A.q' * (B.q * v1 + B.p - A.p)
///    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
B2_INLINE b2Transform b2InvMulTransforms( b2Transform A, b2Transform B )
{
	b2Transform C;
	C.q = b2InvMulRot( A.q, B.q );
	C.p = b2InvRotateVector( A.q, b2Sub( B.p, A.p ) );
	return C;
}

/// Multiply a 2-by-2 matrix times a 2D vector
B2_INLINE b2Vec2 b2MulMV( b2Mat22 A, b2Vec2 v )
{
	b2Vec2 u = {
		A.cx.x * v.x + A.cy.x * v.y,
		A.cx.y * v.x + A.cy.y * v.y,
	};
	return u;
}

/// Get the inverse of a 2-by-2 matrix
B2_INLINE b2Mat22 b2GetInverse22( b2Mat22 A )
{
	float a = A.cx.x, b = A.cy.x, c = A.cx.y, d = A.cy.y;
	float det = a * d - b * c;
	if ( det != 0.0f )
	{
		det = 1.0f / det;
	}

		b2Mat22 B = {
			{ det * d, -det * c },
			{ -det * b, det * a },
		};
		return B;
	}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
B2_INLINE b2Vec2 b2Solve22( b2Mat22 A, b2Vec2 b )
{
	float a11 = A.cx.x, a12 = A.cy.x, a21 = A.cx.y, a22 = A.cy.y;
	float det = a11 * a22 - a12 * a21;
	if ( det != 0.0f )
	{
		det = 1.0f / det;
	}
	b2Vec2 x = { det * ( a22 * b.x - a12 * b.y ), det * ( a11 * b.y - a21 * b.x ) };
	return x;
}

/// Does a fully contain b
B2_INLINE bool b2AABB_Contains( b2AABB a, b2AABB b )
{
	bool s = true;
	s = s && a.lowerBound.x <= b.lowerBound.x;
	s = s && a.lowerBound.y <= b.lowerBound.y;
	s = s && b.upperBound.x <= a.upperBound.x;
	s = s && b.upperBound.y <= a.upperBound.y;
	return s;
}

/// Get the center of the AABB.
B2_INLINE b2Vec2 b2AABB_Center( b2AABB a )
{
	b2Vec2 b = { 0.5f * ( a.lowerBound.x + a.upperBound.x ), 0.5f * ( a.lowerBound.y + a.upperBound.y ) };
	return b;
}

/// Get the extents of the AABB (half-widths).
B2_INLINE b2Vec2 b2AABB_Extents( b2AABB a )
{
	b2Vec2 b = { 0.5f * ( a.upperBound.x - a.lowerBound.x ), 0.5f * ( a.upperBound.y - a.lowerBound.y ) };
	return b;
}

/// Union of two AABBs
B2_INLINE b2AABB b2AABB_Union( b2AABB a, b2AABB b )
{
	b2AABB c;
	c.lowerBound.x = b2MinFloat( a.lowerBound.x, b.lowerBound.x );
	c.lowerBound.y = b2MinFloat( a.lowerBound.y, b.lowerBound.y );
	c.upperBound.x = b2MaxFloat( a.upperBound.x, b.upperBound.x );
	c.upperBound.y = b2MaxFloat( a.upperBound.y, b.upperBound.y );
	return c;
}

/// Do a and b overlap
B2_INLINE bool b2AABB_Overlaps( b2AABB a, b2AABB b )
{
	return !( b.lowerBound.x > a.upperBound.x || b.lowerBound.y > a.upperBound.y || a.lowerBound.x > b.upperBound.x ||
			  a.lowerBound.y > b.upperBound.y );
}

/// Compute the bounding box of an array of circles
B2_INLINE b2AABB b2MakeAABB( const b2Vec2* points, int count, float radius )
{
	B2_ASSERT( count > 0 );
	b2AABB a = { points[0], points[0] };
	for ( int i = 1; i < count; ++i )
	{
		a.lowerBound = b2Min( a.lowerBound, points[i] );
		a.upperBound = b2Max( a.upperBound, points[i] );
	}

	b2Vec2 r = { radius, radius };
	a.lowerBound = b2Sub( a.lowerBound, r );
	a.upperBound = b2Add( a.upperBound, r );

	return a;
}

/// Signed separation of a point from a plane
B2_INLINE float b2PlaneSeparation( b2Plane plane, b2Vec2 point )
{
	return b2Dot( plane.normal, point ) - plane.offset;
}

/// One-dimensional mass-spring-damper simulation. Returns the new velocity given the position and time step.
/// You can then compute the new position using:
/// position += timeStep * newVelocity
/// This drives towards a zero position. By using implicit integration we get a stable solution
/// that doesn't require transcendental functions.
B2_INLINE float b2SpringDamper( float hertz, float dampingRatio, float position, float velocity, float timeStep )
{
	float omega = 2.0f * B2_PI * hertz;
	float omegaH = omega * timeStep;
	return ( velocity - omega * omegaH * position ) / ( 1.0f + 2.0f * dampingRatio * omegaH + omegaH * omegaH );
}

/// Box2D bases all length units on meters, but you may need different units for your game.
/// You can set this value to use different units. This should be done at application startup
/// and only modified once. Default value is 1.
/// For example, if your game uses pixels for units you can use pixels for all length values
/// sent to Box2D. There should be no extra cost. However, Box2D has some internal tolerances
/// and thresholds that have been tuned for meters. By calling this function, Box2D is able
/// to adjust those tolerances and thresholds to improve accuracy.
/// A good rule of thumb is to pass the height of your player character to this function. So
/// if your player character is 32 pixels high, then pass 32 to this function. Then you may
/// confidently use pixels for all the length values sent to Box2D. All length values returned
/// from Box2D will also be pixels because Box2D does not do any scaling internally.
/// However, you are now on the hook for coming up with good values for gravity, density, and
/// forces.
/// @warning This must be modified before any calls to Box2D
B2_API void b2SetLengthUnitsPerMeter( float lengthUnits );

/// Get the current length units per meter.
B2_API float b2GetLengthUnitsPerMeter( void );

/**@}*/

/**
 * @defgroup math_cpp C++ Math
 * @brief Math operator overloads for C++
 *
 * See math_functions.h for details.
 * @{
 */

#ifdef __cplusplus

/// Unary add one vector to another
inline void operator+=( b2Vec2& a, b2Vec2 b )
{
	a.x += b.x;
	a.y += b.y;
}

/// Unary subtract one vector from another
inline void operator-=( b2Vec2& a, b2Vec2 b )
{
	a.x -= b.x;
	a.y -= b.y;
}

/// Unary multiply a vector by a scalar
inline void operator*=( b2Vec2& a, float b )
{
	a.x *= b;
	a.y *= b;
}

/// Unary negate a vector
inline b2Vec2 operator-( b2Vec2 a )
{
	return { -a.x, -a.y };
}

/// Binary vector addition
inline b2Vec2 operator+( b2Vec2 a, b2Vec2 b )
{
	return { a.x + b.x, a.y + b.y };
}

/// Binary vector subtraction
inline b2Vec2 operator-( b2Vec2 a, b2Vec2 b )
{
	return { a.x - b.x, a.y - b.y };
}

/// Binary scalar and vector multiplication
inline b2Vec2 operator*( float a, b2Vec2 b )
{
	return { a * b.x, a * b.y };
}

/// Binary scalar and vector multiplication
inline b2Vec2 operator*( b2Vec2 a, float b )
{
	return { a.x * b, a.y * b };
}

/// Binary vector equality
inline bool operator==( b2Vec2 a, b2Vec2 b )
{
	return a.x == b.x && a.y == b.y;
}

/// Binary vector inequality
inline bool operator!=( b2Vec2 a, b2Vec2 b )
{
	return a.x != b.x || a.y != b.y;
}

#endif

/**@}*/