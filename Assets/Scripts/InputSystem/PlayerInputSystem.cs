using System;
using UnityEngine;
using UnityEngine.InputSystem;

namespace ForkliftDemo.InputSystem
{
    public class PlayerInputSystem : MonoBehaviour
    {
        private InputActions input;
        public event Action<float> OnLiftPerformed;
        public event Action<Vector2> OnDrivePerformed;

        private void Awake()
        {
            input = new InputActions();
            input.Forklift.Drive.performed += InvokeOnDrivePerformed;
            input.Forklift.Lift.performed += InvokeOnLiftPerformed;
        }

        private void OnDestroy()
        {
            input.Forklift.Drive.performed -= InvokeOnDrivePerformed;
            input.Forklift.Lift.performed -= InvokeOnLiftPerformed;
        }

        private void InvokeOnLiftPerformed(InputAction.CallbackContext callbackContext)
        {
            var value = callbackContext.ReadValue<float>();
            OnLiftPerformed?.Invoke(value);
        }

        private void InvokeOnDrivePerformed(InputAction.CallbackContext callbackContext)
        {
            var value = callbackContext.ReadValue<Vector2>();
            OnDrivePerformed?.Invoke(value);
        }
    }
}