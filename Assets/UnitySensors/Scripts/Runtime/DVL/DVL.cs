using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace FRJ.Sensor
{
    [RequireComponent(typeof(Rigidbody))]
    public class DVL : MonoBehaviour
    {
        private Rigidbody _rb;
        private Transform _trans;

        // Previous value
        private Vector3 _lastVelocity = Vector3.zero;

        private Vector4 _geometryQuaternion;
        private Vector3 _angularVelocity;
        private Vector3 _linearVelocity;

        private Noise.Gaussian gaussianNoise;
        private Noise.Bias biasNoise;

        [SerializeField] private float _scanRate = 100f;
        public float scanRate { get => this._scanRate; }

        public bool enableGaussianNoise;
        public bool enableBiasNoise;

        public NoiseSetting setting;

        public Vector4 GeometryQuaternion { get => _geometryQuaternion; }
        public Vector3 AngularVelocity { get => _angularVelocity; }
        public Vector3 LinearVelocity { get => _linearVelocity; }

        [System.Serializable]
        public class NoiseSetting
        {
            public Vector3 angVelSigma;
            public Vector3 angVelBias;
            public Vector3 linVelSigma;
            public Vector3 linVelBias;
        }

        private void Start()
        {
            this._trans = this.GetComponent<Transform>();
            this._rb = this.GetComponent<Rigidbody>();
            this._angularVelocity = new Vector3();
            this._linearVelocity = new Vector3();
            this.gaussianNoise = new Noise.Gaussian();
        }

        public void UpdateDVL()
        {
            // Update Object State //
            // Calculate Move Element
            Vector3 localLinearVelocity = this._trans.InverseTransformDirection(this._rb.velocity);
            this._lastVelocity = localLinearVelocity;

            // Raw
            // this._angularVelocity = new Vector3();
            this._angularVelocity = -1 * this.transform.InverseTransformVector(this.GetComponent<Rigidbody>().angularVelocity);
            this._linearVelocity = localLinearVelocity;

            // Apply Gaussian Noise
            // if (this.enableGaussianNoise) { this._angularVelocity = this.gaussianNoise.Apply(this._angularVelocity, this.setting.angVelSigma); }
            // if (this.enableGaussianNoise) { this._linearVelocity = this.gaussianNoise.Apply(this._linearVelocity, this.setting.linVelSigma); }

            // Apply Bias Noise
            // if (this.enableBiasNoise) { this._angularVelocity = this.biasNoise.Apply(this._angularVelocity, this.setting.angVelSigma); }
            // if (this.enableBiasNoise) { this._linearAcceleration = this.biasNoise.Apply(this._linearAcceleration, this.setting.linAccSigma); }
        }

#if UNITY_EDITOR
        [CustomEditor(typeof(DVL))]
        public class DVLEditor : Editor
        {
            private DVL variables;

            private void Awake()
            {
                this.variables = target as DVL;
            }

            // inspector��GUI�ݒ�
            public override void OnInspectorGUI()
            {
                EditorGUI.BeginChangeCheck();

                this.variables.enableGaussianNoise = EditorGUILayout.ToggleLeft("Enable Gaussian Noise", this.variables.enableGaussianNoise);
                if (this.variables.enableGaussianNoise)
                {
                    EditorGUILayout.LabelField("Gaussian Noise Setting");
                    this.variables.setting.angVelSigma = EditorGUILayout.Vector3Field("->AngularVelocity Sigma", this.variables.setting.angVelSigma);
                    this.variables.setting.linVelSigma = EditorGUILayout.Vector3Field("->LinearAcceleration Sigma", this.variables.setting.linVelSigma);
                }
                this.variables.enableBiasNoise = EditorGUILayout.ToggleLeft("Enable Bias Noise", this.variables.enableBiasNoise);
                if (this.variables.enableBiasNoise)
                {
                    EditorGUILayout.LabelField("Bias Noise Setting");
                    this.variables.setting.angVelBias = EditorGUILayout.Vector3Field("->AngularVelocity Bias", this.variables.setting.angVelBias);
                    this.variables.setting.linVelBias = EditorGUILayout.Vector3Field("->LinearAcceleration Bias", this.variables.setting.linVelBias);
                }

                // GUI�̍X�V������������s
                if (EditorGUI.EndChangeCheck())
                {
                    EditorUtility.SetDirty(this.variables);
                }
            }
        }
#endif

    }
}
