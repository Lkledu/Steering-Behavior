using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SteeringBehaviors : MonoBehaviour
{
    [Header("Physics")]
    public float m_Mass = 15.0f;

    [Header("Seek and Flee")]
    public float m_MaxVelocity = 5.0f;
    public float m_MaxForce = 15.0f;
    public Boid m_Target;

    [Header("Pursuit and Evade")]
    public float m_Prediction = 3.0f;

    [Header("Arrive")]
    public float m_SlowingRadius = 5.0f;

    [Header("Wander")]
    public float m_WanderDistance = 5.0f;
    public float m_WanderRadius = 1.0f;
    public float m_WanderRate = 0.2f;

    [Header("Obstacle Avoidance")]
    public bool m_UseObstacleAvoidance;
    public float[] m_AheadAngles;
    public float m_MaxSeeAhead = 5.0f;
    public LayerMask m_ObstacleLayer;
    public float m_AvoidanceForce = 10.0f;

    [Header("Path Follow")]
    public List<Transform> m_Waypoint;
    public float m_WaypointRadius;
    public enum PathFollowMode { Reset, Arrive, PingPong, Random };
    public PathFollowMode m_PathFollowMode = PathFollowMode.Reset;

    private int m_CurrentWaypoint;
    private int m_WaypointStep = 1;

    [Header("Mode")]
    public SteerMode m_SteerMode = SteerMode.None;
    public enum SteerMode { None, Seek, Flee, Arrive, Pursuit, Evade, Wander, PathFollow, Flock };

    private Vector3 m_Velocity;
    private Rigidbody m_Body;

    private void Start()
    {
        m_Body = GetComponent<Rigidbody>();
        m_Body.mass = m_Mass;
        m_Velocity = m_Body.transform.forward;
    }
    
    public Vector3 Seek(Vector3 targetPosition)
    {
        Vector3 desiredVelocity = targetPosition - m_Body.position;
        desiredVelocity = desiredVelocity.normalized * m_MaxVelocity;
        Vector3 steeringForce = desiredVelocity - m_Velocity;

        Debug.DrawRay(m_Body.transform.position, desiredVelocity.normalized * 5.0f, Color.red);

        return steeringForce;
    }

    public Vector3 Pursuit(Vector3 targetPosition, Vector3 targetVelocity)
    {
        Vector3 futurePosition = targetPosition + targetVelocity * m_Prediction;
        return Seek(futurePosition);
    }

    public Vector3 Evade(Vector3 targetPosition, Vector3 targetVelocity)
    {
        Vector3 futurePosition = targetPosition + targetVelocity * m_Prediction;
        return Flee(futurePosition);
    }

    public Vector3 Wander()
    {
        Vector3 circleCenter = transform.forward.normalized * m_WanderDistance;
        Vector2 randomPoint = Random.insideUnitCircle;

        Vector3 displacement = new Vector3(randomPoint.x, 0.0f ,randomPoint.y) * m_WanderRadius;
        displacement = Quaternion.LookRotation(transform.forward.normalized) * displacement;

        Vector3 target = transform.position + circleCenter + displacement;
        return Seek(target);
    }

    public Vector3 Arrive(Vector3 targetPosition, float slowingRadius)
    {
        Vector3 desiredVelocity = targetPosition - m_Body.position;

        float distance = desiredVelocity.magnitude;

        desiredVelocity = desiredVelocity.normalized * m_MaxVelocity;
        
        if (distance < slowingRadius)
        {
            desiredVelocity *= (distance / slowingRadius);
        }
        
        Vector3 steeringForce = desiredVelocity - m_Velocity;

        Debug.DrawRay(m_Body.transform.position, desiredVelocity.normalized * 5.0f, Color.red);

        return steeringForce;
    }

    public Vector3 Flee(Vector3 targetPosition)
    {
        Vector3 desiredVelocity = m_Body.position - targetPosition;
        desiredVelocity = desiredVelocity.normalized * m_MaxVelocity;
        Vector3 steeringForce = desiredVelocity - m_Velocity;

        Debug.DrawRay(m_Body.transform.position, desiredVelocity.normalized * 5.0f, Color.red);

        return steeringForce;
    }

    public Vector3 PathFollow()
    {
        Vector3 target = m_Waypoint[m_CurrentWaypoint].position;
        if (Vector3.Distance(m_Body.transform.position, target) <= m_WaypointRadius)
        {
            switch (m_PathFollowMode)
            {
                case PathFollowMode.Reset:
                    m_CurrentWaypoint = ++m_CurrentWaypoint % m_Waypoint.Count;
                    break;
                case PathFollowMode.Arrive:
                    m_CurrentWaypoint = Mathf.Min(++m_CurrentWaypoint, m_Waypoint.Count - 1);
                    break;
                case PathFollowMode.PingPong:
                    m_CurrentWaypoint += m_WaypointStep;
                    if (m_CurrentWaypoint >= m_Waypoint.Count || m_CurrentWaypoint < 0)
                    {
                        m_WaypointStep *= -1;
                        m_CurrentWaypoint += m_WaypointStep;
                    }
                    break;
                case PathFollowMode.Random:
                    m_CurrentWaypoint = Random.Range(0, m_Waypoint.Count);
                    break;
                default:
                    break;
            }
        }

        return m_PathFollowMode == PathFollowMode.Arrive && m_CurrentWaypoint == m_Waypoint.Count - 1? 
            Arrive(target, m_SlowingRadius) : Seek(target);
    }

    public Vector3 CollisionAvoidance(float angle)
    {
        Vector3 direction = Quaternion.AngleAxis(angle, transform.up) * transform.forward;

        RaycastHit hit;
        if (Physics.Raycast(transform.position, direction, out hit, m_MaxSeeAhead, m_ObstacleLayer))
        {
            float force = 1.0f - (hit.distance / m_MaxSeeAhead) * m_AvoidanceForce;
            Vector3 directionForce = Vector3.Reflect(direction, hit.normal) * -1 * force;
            directionForce.y = 0.0f;
            return directionForce;
        }

        Debug.DrawLine(transform.position, transform.position + direction * m_MaxSeeAhead, Color.green);
        
        return Vector3.zero;
    }

    public Vector3 Alignment(List<Rigidbody> nearbyBoids) {
        Vector3 alignment = transform.forward;

        foreach (var boid in nearbyBoids) {
            alignment += boid.transform.forward;
        }

        return alignment / nearbyBoids.Count;
    }

    public Vector3 Cohesion(List<Rigidbody> nearbyBoids) {
        Vector3 alignment = transform.forward;

        foreach (var boid in nearbyBoids)
        {
            alignment += boid.transform.position;
        }

        return alignment / (float)nearbyBoids.Count;
    }

    public Vector3 Separation(List<Rigidbody> nearbyBoids) {

        Vector3 separation = Vector3.zero;
        if (nearbyBoids.Count == 1) {
            return separation;
        }

        foreach (var boid in nearbyBoids)
        {
            Vector3 direction = transform.position - boid.position;
            separation += direction.normalized;
        }

        return separation / (float)nearbyBoids.Count;
    }
    
    [Header("Flock")]
    public float m_FlockRadius = 5;
    [Range(0.0f, 1.0f)]
    public float m_AlignmentRate;
    [Range(0.0f, 1.0f)]
    public float m_CohesiontRate;
    [Range(0.0f, 1.0f)]
    public float m_SeparationRate;

    public Vector3 m_TankCenter = Vector3.zero;
    public float m_TankRadius = 5.0f;
    public LayerMask m_FlockingLayer;

    public Vector3 Flock() {
        Collider[] boidColliders = Physics.OverlapSphere(transform.position, m_FlockRadius, m_FlockingLayer);
        List<Rigidbody> nearbyBoids = new List<Rigidbody>();
        foreach (var boid in boidColliders) {
            nearbyBoids.Add(boid.GetComponent<Rigidbody>());
        }


        GameObject rb = new GameObject();
        Vector3 separation = Separation(nearbyBoids);
        Vector3 alignment = Alignment(nearbyBoids);
        Vector3 cohesion = Cohesion(nearbyBoids);

        var flocking = separation * m_SeparationRate
                        + alignment * m_AlignmentRate
                        + cohesion * m_CohesiontRate;

        return flocking;
    }



    public bool VerifyTankBoundary(float radius) {
        return Vector3.Distance(transform.position, m_TankCenter) > radius;
    }

    public void ApplyForce(Vector3 steeringForce)
    {
        if (m_UseObstacleAvoidance)
        {
            for (int i = 0; i < m_AheadAngles.Length; i++)
            {
                steeringForce += CollisionAvoidance(m_AheadAngles[i]);
            }
        }

        steeringForce = Vector3.ClampMagnitude(steeringForce, m_MaxForce);
        steeringForce /= m_Mass;

        m_Velocity = Vector3.ClampMagnitude(m_Velocity + steeringForce, m_MaxVelocity);

        m_Body.MovePosition(m_Body.transform.position + m_Velocity * Time.deltaTime);
        if (m_Velocity.magnitude != 0)
            m_Body.MoveRotation(Quaternion.LookRotation(m_Velocity.normalized));
        

        Debug.DrawRay(m_Body.transform.position, m_Velocity.normalized * 5.0f, Color.blue);
    }

    private void FixedUpdate()
    {
        switch (m_SteerMode)
        {
            case SteerMode.None:
                break;
            case SteerMode.Seek:
                ApplyForce(Seek(m_Target.Position));
                break;
            case SteerMode.Flee:
                ApplyForce(Flee(m_Target.Position));
                break;
            case SteerMode.Arrive:
                ApplyForce(Arrive(m_Target.Position, m_SlowingRadius));
                break;
            case SteerMode.Pursuit:
                ApplyForce(Pursuit(m_Target.Position, m_Target.Velocity));
                break;
            case SteerMode.Evade:
                ApplyForce(Evade(m_Target.Position, m_Target.Velocity));
                break;
            case SteerMode.Wander:
                ApplyForce(Wander());
                break;
            case SteerMode.PathFollow:
                ApplyForce(PathFollow());
                break;
            case SteerMode.Flock:

                m_Body.AddForce(-Physics.gravity, ForceMode.Acceleration);

                if (VerifyTankBoundary(m_TankRadius))
                {
                    ApplyForce(Seek(m_TankCenter));
                }
                else {
                    var flock = Flock();
                    var wander = Wander();
                    ApplyForce(Seek(flock + wander));
                }
                break;
            default:
                break;
        }
    }

    private void Awake()
    {
        m_Body = GetComponent<Rigidbody>();
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(m_TankCenter, m_TankRadius);
    }
}
