using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class test : MonoBehaviour
{

    public GameObject gm;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 toTr = gm.transform.position - transform.position;
        float angleToLookahead = Vector3.SignedAngle(transform.forward, toTr, Vector3.up) ;

        float maxStep = 90 * Time.deltaTime;
        float step = Mathf.Clamp(angleToLookahead, -maxStep, maxStep);

        gameObject.transform.Rotate(0, angleToLookahead, 0);
    }
}
