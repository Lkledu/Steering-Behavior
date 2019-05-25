using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Boid : MonoBehaviour
{
    public Vector3 m_Direction;
    public float m_Speed;
    private Rigidbody m_Body;

    private void Start()
    {
        m_Body = GetComponent<Rigidbody>();  
    }

    public void FixedUpdate()
    {
        m_Body.velocity = transform.InverseTransformDirection(m_Direction) * m_Speed;
    }

    public Vector3 Velocity => transform.forward; 

    public Vector3 Position => transform.position;
}
