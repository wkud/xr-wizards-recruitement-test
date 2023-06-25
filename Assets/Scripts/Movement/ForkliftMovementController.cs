using ForkliftDemo.InputSystem;
using UnityEngine;
using UnityEngine.Serialization;

namespace ForkliftDemo.Movement
{
    public class ForkliftMovementController : MonoBehaviour
    {
        [FormerlySerializedAs("inputSystem")]
        [SerializeField]
        private PlayerInputSystem playerInputSystem;

        private void Awake()
        {
            playerInputSystem.OnLiftPerformed += OnLiftPerformed;
            playerInputSystem.OnDrivePerformed += OnDrivePerformed;
        }

        private void OnDestroy()
        {
            playerInputSystem.OnLiftPerformed -= OnLiftPerformed;
            playerInputSystem.OnDrivePerformed -= OnDrivePerformed;
        }

        private void OnLiftPerformed(float value)
        {

        }

        private void OnDrivePerformed(Vector2 value)
        {

        }
    }
}
