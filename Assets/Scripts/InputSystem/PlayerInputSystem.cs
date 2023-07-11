using UnityEngine;
using UnityEngine.InputSystem;

namespace ForkliftDemo.InputSystem
{
    public class PlayerInputSystem : MonoBehaviour
    {
        private InputActions input;

        public float LiftingInputValue => input.Forklift.Lift.ReadValue<float>();
        public Vector2 DrivingInputValue => input.Forklift.Drive.ReadValue<Vector2>();

        private void Awake()
        {
            input = new InputActions();
            input.Forklift.Enable();
        }
    }
}