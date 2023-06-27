using UnityEngine;

namespace ForkliftDemo.ExtensionMethods
{
    public static class FloatExtension
    {
        public static float Map(this float oldValue, float newMin, float newMax, float oldMin = 0, float oldMax = 1)
        {
            var normalizedValue = NormalizeFromRange(oldValue, oldMin, oldMax);

            var newRange = Mathf.Abs(newMin - newMax);
            var newValue = normalizedValue * newRange + newMin;
            return newValue;
        }

        private static float NormalizeFromRange(float oldValue, float oldMin, float oldMax)
        {
            if (oldMin == 0 && oldMax == 1)
            {
                return oldValue;
            }

            var oldRange = Mathf.Abs(oldMin - oldMax);
            var normalizedValue = (oldValue - oldMin) / oldRange; // value from 0 to 1
            return normalizedValue;
        }
    }
}