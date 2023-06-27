using System;
using UnityEngine;
using UnityEngine.InputSystem;

namespace ForkliftDemo.InputSystem
{
    public class PlayerInputSystem : MonoBehaviour
    {
        private InputActions input;
        public event Action<float> OnLiftingUpdated;
        public event Action<Vector2> OnDrivingUpdated;

        private bool isDriving;
        private bool isLifting;

        private float lastLiftingInputValue;
        private Vector2 lastDrivingInputValue;

        private void Awake()
        {
            input = new InputActions();
            input.Forklift.Enable();

            input.Forklift.Lift.started += StartLifting;
            input.Forklift.Lift.canceled += StopLifting;

            input.Forklift.Drive.started += StartDriving;
            input.Forklift.Drive.canceled += StopDriving;
        }

        private void OnDestroy()
        {
            input.Forklift.Lift.started -= StartLifting;
            input.Forklift.Lift.canceled -= StopLifting;

            input.Forklift.Drive.started -= StartDriving;
            input.Forklift.Drive.canceled -= StopDriving;
        }

        private void FixedUpdate()
        {
            if (isLifting)
            {
                OnLiftingUpdated?.Invoke(lastLiftingInputValue);
            }

            if (isDriving)
            {
                OnDrivingUpdated?.Invoke(lastDrivingInputValue);
            }
        }

        private void StartLifting(InputAction.CallbackContext callbackContext)
        {
            isLifting = true;
            lastLiftingInputValue = callbackContext.ReadValue<float>();
        }

        private void StopLifting(InputAction.CallbackContext callbackContext) => isLifting = false;


        private void StartDriving(InputAction.CallbackContext callbackContext)
        {
            isDriving = true;
            lastDrivingInputValue = callbackContext.ReadValue<Vector2>();
        }

        private void StopDriving(InputAction.CallbackContext callbackContext) => isDriving = false;
    }
}