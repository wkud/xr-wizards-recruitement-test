using UnityEngine;
using UnityEngine.InputSystem;

namespace ForkliftDemo.InputSystem
{
    public class PlayerInputSystem : MonoBehaviour
    {
        private InputActions input;

        public float LiftingInputValue { get; private set; }
        public Vector2 DrivingInputValue => input.Forklift.Drive.ReadValue<Vector2>();

        private void Awake()
        {
            input = new InputActions();
            input.Forklift.Enable();

            input.Forklift.Lift.started += StartLifting;
            input.Forklift.Lift.canceled += StopLifting;
        }

        private void OnDestroy()
        {
            input.Forklift.Lift.started -= StartLifting;
            input.Forklift.Lift.canceled -= StopLifting;
        }

        private void StartLifting(InputAction.CallbackContext callbackContext)
            => LiftingInputValue = callbackContext.ReadValue<float>();

        private void StopLifting(InputAction.CallbackContext callbackContext)
            => LiftingInputValue = default;

    }
}