class Vector2
{
    // X component of the vector.
    public x: number;
    // Y component of the vector.
    public y: number;

    // Constructs a new vector with given x, y components.
    public constructor(x: number, y: number) { this.x = x; this.y = y; }

    // Set x and y components of an existing Vector2.
    public Set(newX: number, newY: number): void { this.x = newX; this.y = newY; }

    // Linearly interpolates between two vectors.
    public static Lerp(a: Vector2, b: Vector2, t: number): Vector2
    {
        //t = Mathf.Clamp01(t);
        return new Vector2(
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t
        );
    }

    // Linearly interpolates between two vectors without clamping the interpolant
    public static LerpUnclamped(a: Vector2, b: Vector2, t: number): Vector2
    {
        return new Vector2(
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t
        );
    }

    // Moves a point /current/ towards /target/.
    public static MoveTowards(current: Vector2, target: Vector2, maxDistanceDelta: number): Vector2
    {
        // avoid vector ops because current scripting backends are terrible at inlining
        const toVector_x: number = target.x - current.x;
        const toVector_y: number = target.y - current.y;

        const sqDist: number = toVector_x * toVector_x + toVector_y * toVector_y;

        if (sqDist == 0 || (maxDistanceDelta >= 0 && sqDist <= maxDistanceDelta * maxDistanceDelta))
            return target;

        const dist: number = Math.sqrt(sqDist);

        return new Vector2(current.x + toVector_x / dist * maxDistanceDelta,
            current.y + toVector_y / dist * maxDistanceDelta);
    }

    // Multiplies two vectors component-wise.
    public static Scale(a: Vector2, b: Vector2): Vector2 { return new Vector2(a.x * b.x, a.y * b.y); }

    // Makes this vector have a ::ref::magnitude of 1.
    public Normalize(): void
    {
        const mag: number = this.magnitude;
        if (mag > kEpsilon) {
            this.x = this.x / mag;
            this.y = this.y / mag;
        } else {
            this.x = 0;
            this.y = 0;
        }
    }

    // Returns this vector with a ::ref::magnitude of 1 (RO).
    public get normalized(): Vector2
    {
        let v: Vector2 = new Vector2(this.x, this.y);
        v.Normalize();
        return v;
    }

    /// Returns a string version
    public ToString(): string
    {
        return `(${this.x}, ${this.y})`;
    }

    // used to allow Vector2s to be used as keys in hash tables
    /*
    public GetHashCode(): number
    {
        return x.GetHashCode() ^ (y.GetHashCode() << 2);
    }

    // also required for being able to use Vector2s as keys in hash tables
    [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
    public override bool Equals(object other)
    {
        if (other is Vector2 v)
            return Equals(v);
        return false;
    }

    [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
    public bool Equals(Vector2 other)
    {
        return x == other.x && y == other.y;
    }
    */

    public static Reflect(inDirection: Vector2, inNormal: Vector2): Vector2
    {
        const factor: number = -2 * Vector2.Dot(inNormal, inDirection);
        return new Vector2(factor * inNormal.x + inDirection.x, factor * inNormal.y + inDirection.y);
    }

    public static Perpendicular(inDirection: Vector2): Vector2
    {
        return new Vector2(-inDirection.y, inDirection.x);
    }

    // Dot Product of two vectors.
    public static Dot(lhs: Vector2, rhs: Vector2): number { return lhs.x * rhs.x + lhs.y * rhs.y; }

    // Returns the length of this vector (RO).
    public get magnitude(): number { return Math.sqrt(this.x * this.x + this.y * this.y); }
    // Returns the squared length of this vector (RO).
    public get sqrMagnitude(): number { return this.x * this.x + this.y * this.y; }

    // Returns the angle in degrees between /from/ and /to/.
    public static Angle(from: Vector2, to: Vector2): number
    {
        // sqrt(a) * sqrt(b) = sqrt(a * b) -- valid for real numbers
        const denominator: number = Math.sqrt(from.sqrMagnitude * to.sqrMagnitude);
        if (denominator < kEpsilonNormalSqrt)
            return 0;

        const dot: number = Mathf.Clamp(Vector2.Dot(from, to) / denominator, -1, 1);
        return Math.acos(dot) * Mathf.Rad2Deg;
    }

    // Returns the signed angle in degrees between /from/ and /to/. Always returns the smallest possible angle
    public static SignedAngle(from: Vector2, to: Vector2): number
    {
        const unsigned_angle: number = Vector2.Angle(from, to);
        const sign: number = Math.sign(from.x * to.y - from.y * to.x);
        return unsigned_angle * sign;
    }

    // Returns the distance between /a/ and /b/.
    public static Distance(a: Vector2, b: Vector2): number
    {
        const diff_x: number = a.x - b.x;
        const diff_y: number = a.y - b.y;
        return Math.sqrt(diff_x * diff_x + diff_y * diff_y);
    }

    // Returns a copy of /vector/ with its magnitude clamped to /maxLength/.
    public static ClampMagnitude(vector: Vector2, maxLength: number): Vector2
    {
        const sqrMagnitude: number = vector.sqrMagnitude;
        if (sqrMagnitude > maxLength * maxLength)
        {
            const mag: number = Math.sqrt(sqrMagnitude);

            //these intermediate variables force the intermediate result to be
            //of float precision. without this, the intermediate result can be of higher
            //precision, which changes behavior.
            const normalized_x : number= vector.x / mag;
            const normalized_y: number = vector.y / mag;
            return new Vector2(normalized_x * maxLength,
                normalized_y * maxLength);
        }
        return vector;
    }

    // Returns a vector that is made from the smallest components of two vectors.
    public static Min(lhs: Vector2, rhs: Vector2): Vector2 { return new Vector2(Math.min(lhs.x, rhs.x), Math.min(lhs.y, rhs.y)); }

    // Returns a vector that is made from the largest components of two vectors.
    public static Max(lhs: Vector2, rhs: Vector2): Vector2 { return new Vector2(Math.max(lhs.x, rhs.x), Math.max(lhs.y, rhs.y)); }

    public static SmoothDamp(current: Vector2, target: Vector2, currentVelocity: Vector2, smoothTime: number, maxSpeed: number): Vector2;
    public static SmoothDamp(current: Vector2, target: Vector2, currentVelocity: Vector2, smoothTime: number): Vector2;
    public static SmoothDamp(current: Vector2, target: Vector2, currentVelocity: Vector2, smoothTime: number, maxSpeed: number, deltaTime: number): Vector2;

    public static Vector2 SmoothDamp(Vector2 current, Vector2 target, ref Vector2 currentVelocity, float smoothTime, [uei.DefaultValue("Mathf.Infinity")] float maxSpeed, [uei.DefaultValue("Time.deltaTime")] float deltaTime)
    {
        // Based on Game Programming Gems 4 Chapter 1.10
        smoothTime = Mathf.Max(0.0001F, smoothTime);
        float omega = 2F / smoothTime;

        float x = omega * deltaTime;
        float exp = 1F / (1F + x + 0.48F * x * x + 0.235F * x * x * x);

        float change_x = current.x - target.x;
        float change_y = current.y - target.y;
        Vector2 originalTo = target;

        // Clamp maximum speed
        float maxChange = maxSpeed * smoothTime;

        float maxChangeSq = maxChange * maxChange;
        float sqDist = change_x * change_x + change_y * change_y;
        if (sqDist > maxChangeSq)
        {
            var mag = (float)Math.Sqrt(sqDist);
            change_x = change_x / mag * maxChange;
            change_y = change_y / mag * maxChange;
        }

        target.x = current.x - change_x;
        target.y = current.y - change_y;

        float temp_x = (currentVelocity.x + omega * change_x) * deltaTime;
        float temp_y = (currentVelocity.y + omega * change_y) * deltaTime;

        currentVelocity.x = (currentVelocity.x - omega * temp_x) * exp;
        currentVelocity.y = (currentVelocity.y - omega * temp_y) * exp;

        float output_x = target.x + (change_x + temp_x) * exp;
        float output_y = target.y + (change_y + temp_y) * exp;

        // Prevent overshooting
        float origMinusCurrent_x = originalTo.x - current.x;
        float origMinusCurrent_y = originalTo.y - current.y;
        float outMinusOrig_x = output_x - originalTo.x;
        float outMinusOrig_y = output_y - originalTo.y;

        if (origMinusCurrent_x * outMinusOrig_x + origMinusCurrent_y * outMinusOrig_y > 0)
        {
            output_x = originalTo.x;
            output_y = originalTo.y;

            currentVelocity.x = (output_x - originalTo.x) / deltaTime;
            currentVelocity.y = (output_y - originalTo.y) / deltaTime;
        }
        return new Vector2(output_x, output_y);
    }

    // Adds two vectors.
    [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
    public static Vector2 operator+(Vector2 a, Vector2 b) { return new Vector2(a.x + b.x, a.y + b.y); }
    // Subtracts one vector from another.
    [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
    public static Vector2 operator-(Vector2 a, Vector2 b) { return new Vector2(a.x - b.x, a.y - b.y); }
    // Multiplies one vector by another.
    [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
    public static Vector2 operator*(Vector2 a, Vector2 b) { return new Vector2(a.x * b.x, a.y * b.y); }
    // Divides one vector over another.
    [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
    public static Vector2 operator/(Vector2 a, Vector2 b) { return new Vector2(a.x / b.x, a.y / b.y); }
    // Negates a vector.
    [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
    public static Vector2 operator-(Vector2 a) { return new Vector2(-a.x, -a.y); }
    // Multiplies a vector by a number.
    [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
    public static Vector2 operator*(Vector2 a, float d) { return new Vector2(a.x * d, a.y * d); }
    // Multiplies a vector by a number.
    [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
    public static Vector2 operator*(float d, Vector2 a) { return new Vector2(a.x * d, a.y * d); }
    // Divides a vector by a number.
    [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
    public static Vector2 operator/(Vector2 a, float d) { return new Vector2(a.x / d, a.y / d); }
    // Returns true if the vectors are equal.
    [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
    public static bool operator==(Vector2 lhs, Vector2 rhs)
    {
        // Returns false in the presence of NaN values.
        float diff_x = lhs.x - rhs.x;
        float diff_y = lhs.y - rhs.y;
        return (diff_x * diff_x + diff_y * diff_y) < kEpsilon * kEpsilon;
    }

    // Returns true if vectors are different.
    [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
    public static bool operator!=(Vector2 lhs, Vector2 rhs)
    {
        // Returns true in the presence of NaN values.
        return !(lhs == rhs);
    }

    // Converts a [[Vector3]] to a Vector2.
    [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
    public static implicit operator Vector2(Vector3 v)
    {
        return new Vector2(v.x, v.y);
    }

    // Converts a Vector2 to a [[Vector3]].
    [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
    public static implicit operator Vector3(Vector2 v)
    {
        return new Vector3(v.x, v.y, 0);
    }

    static readonly Vector2 zeroVector = new Vector2(0F, 0F);
    static readonly Vector2 oneVector = new Vector2(1F, 1F);
    static readonly Vector2 upVector = new Vector2(0F, 1F);
    static readonly Vector2 downVector = new Vector2(0F, -1F);
    static readonly Vector2 leftVector = new Vector2(-1F, 0F);
    static readonly Vector2 rightVector = new Vector2(1F, 0F);
    static readonly Vector2 positiveInfinityVector = new Vector2(float.PositiveInfinity, float.PositiveInfinity);
    static readonly Vector2 negativeInfinityVector = new Vector2(float.NegativeInfinity, float.NegativeInfinity);


    // Shorthand for writing @@Vector2(0, 0)@@
    public static Vector2 zero { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return zeroVector; } }
    // Shorthand for writing @@Vector2(1, 1)@@
    public static Vector2 one { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return oneVector; }   }
    // Shorthand for writing @@Vector2(0, 1)@@
    public static Vector2 up { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return upVector; } }
    // Shorthand for writing @@Vector2(0, -1)@@
    public static Vector2 down { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return downVector; } }
    // Shorthand for writing @@Vector2(-1, 0)@@
    public static Vector2 left { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return leftVector; } }
    // Shorthand for writing @@Vector2(1, 0)@@
    public static Vector2 right { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return rightVector; } }
    // Shorthand for writing @@Vector2(float.PositiveInfinity, float.PositiveInfinity)@@
    public static Vector2 positiveInfinity { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return positiveInfinityVector; } }
    // Shorthand for writing @@Vector2(float.NegativeInfinity, float.NegativeInfinity)@@
    public static Vector2 negativeInfinity { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return negativeInfinityVector; } }

    // *Undocumented*
    public const float kEpsilon = 0.00001F;
    // *Undocumented*
    public const float kEpsilonNormalSqrt = 1e-15f;
}