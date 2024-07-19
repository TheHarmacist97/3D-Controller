using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
public class TrackSpeed : MonoBehaviour
{
    public Rigidbody character;
    private TextMeshProUGUI speedText;
    // Start is called before the first frame update
    void Start()
    {
        speedText = GetComponent<TextMeshProUGUI>();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        Vector3 planar = new(character.velocity.x, 0f, character.velocity.z);
        speedText.text = planar.magnitude.ToString("0.00");
    }
}
