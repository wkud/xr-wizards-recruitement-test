using ForkliftDemo.InputSystem;
using UnityEngine;

namespace ForkliftDemo.Movement
{
    class ForkliftLiftingController : MonoBehaviour
    {
        [SerializeField]
        private PlayerInputSystem playerInputSystem;

        [SerializeField]
        private float maxHeightOffsetUpwards;
        [SerializeField]
        private float maxHeightOffsetDownwards;
        [SerializeField]
        private float liftSpeed;

        private float minLiftHeight => originLiftLocalPosition.y - maxHeightOffsetDownwards;
        private float maxLiftHeight => originLiftLocalPosition.y + maxHeightOffsetUpwards;

        private Vector3 originLiftLocalPosition;

        private void Awake()
        {
            originLiftLocalPosition = transform.localPosition;
        }

        private void FixedUpdate()
        {
            HandleLifting(playerInputSystem.LiftingInputValue);
        }

        private void HandleLifting(float liftingInput)
        {
            var liftHeight = transform.localPosition.y;
            liftHeight = liftHeight + liftSpeed * liftingInput * Time.fixedDeltaTime;
            liftHeight = Mathf.Clamp(liftHeight, minLiftHeight, maxLiftHeight);

            var localPosition = transform.localPosition;
            transform.localPosition = new Vector3(localPosition.x, liftHeight, localPosition.z);
        }
    }
}
