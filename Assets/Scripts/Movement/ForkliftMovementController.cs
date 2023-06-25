using ForkliftDemo.InputSystem;
using UnityEngine;

namespace ForkliftDemo.Movement
{
    public class ForkliftMovementController : MonoBehaviour
    {
        [SerializeField]
        private PlayerInputSystem inputSystem;

        private void Awake()
        {
            inputSystem.OnLiftPerformed += OnLiftPerformed;
            inputSystem.OnDrivePerformed += OnDrivePerformed;
        }

        private void OnDestroy()
        {
            inputSystem.OnLiftPerformed -= OnLiftPerformed;
            inputSystem.OnDrivePerformed -= OnDrivePerformed;
        }

        private void OnLiftPerformed(float value)
        {

        }

        private void OnDrivePerformed(Vector2 value)
        {

        }
    }
}
