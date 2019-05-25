using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FlockInstantiator : MonoBehaviour
{
    public float m_Radius = 9;
    public int m_Number = 20;
    public GameObject m_Boid;

    private void Start()
    {
        for (int i = 0; i < m_Number; i++)
        {
            Vector3 position = Random.insideUnitSphere * m_Radius;
            Quaternion rotation = Quaternion.Euler(Random.insideUnitSphere * 360.0f);
            Instantiate(m_Boid, position, rotation);
        }
    }

}
