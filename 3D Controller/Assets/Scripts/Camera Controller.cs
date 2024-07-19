using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Cinemachine;


public class CameraController : MonoBehaviour
{
    [SerializeField] private CinemachineVirtualCamera followCam;
    [SerializeField] private CinemachineVirtualCamera aimCamRight;
    [SerializeField] private CinemachineVirtualCamera aimCamLeft;
    [SerializeField] private CanvasGroup reticleGroup;
    [SerializeField] private float duration;

    private bool isAiming, lastAimState;
    private bool rightShoulder;
    private Coroutine aimCoroutine;

    // Update is called once per frame
    private void Start()
    {
        DiableAllCameras();

        SetState();
    }

    private void DiableAllCameras()
    {
        followCam.enabled = false;
        aimCamRight.enabled = false;
        aimCamLeft.enabled = false;
    }

    public void SwitchFollowType(bool isAiming)
    {
        this.isAiming = isAiming;
        SetState();
    }

    public void SwitchShoulder(bool rightShoulder)
    {
        this.rightShoulder = rightShoulder;
        SetState();
    }

    private void SetState()
    {
        followCam.enabled = !isAiming;
        aimCamRight.enabled = isAiming && rightShoulder;
        aimCamLeft.enabled = isAiming && !rightShoulder;

        if (lastAimState != isAiming)
        {
            if (aimCoroutine != null)
                StopCoroutine(aimCoroutine);

            aimCoroutine = StartCoroutine(LerpReticle());
        }
        lastAimState = isAiming;
    }

    private IEnumerator LerpReticle()
    {
        float target = isAiming ? 1.0f : 0.0f;
        float current = reticleGroup.alpha;

        if (current == target) yield break;
        float elapsedTime = 0f;
        
        while(elapsedTime < 1f)
        {
            elapsedTime += Time.deltaTime / duration;
            reticleGroup.alpha = Mathf.Lerp(current, target, elapsedTime);
            yield return null;
        }
    }
}
