using UnityEngine;
using UnityEngine.InputSystem;

namespace ForkliftDemo.InputSystem
{
    public class PlayerInputSystem : MonoBehaviour
    {
        private InputActions input;

        public float LiftingInputValue { get; private set; }
        public Vector2 DrivingInputValue { get; private set; }

        private void Awake()
        {
            input = new InputActions();
            input.Forklift.Enable();

            input.Forklift.Lift.started += StartLifting;
            input.Forklift.Drive.started += StartDriving;
        }

        private void OnDestroy()
        {
            input.Forklift.Lift.started -= StartLifting;
            input.Forklift.Drive.started -= StartDriving;
        }

        private void StartLifting(InputAction.CallbackContext callbackContext)
        {
            LiftingInputValue = callbackContext.ReadValue<float>();
        }

        private void StartDriving(InputAction.CallbackContext callbackContext)
        {
            DrivingInputValue = callbackContext.ReadValue<Vector2>();
        }
    }
}