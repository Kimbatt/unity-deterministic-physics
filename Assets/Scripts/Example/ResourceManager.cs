using UnityEngine;

public class ResourceManager : MonoBehaviour
{
    public Material defaultMaterial;
    public int seed;

    private static ResourceManager _instance;
    public static ResourceManager Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = FindObjectOfType<ResourceManager>();
            }

            return _instance;
        }
    }

    public static MaterialPropertyBlock materialPropertyBlock;

    private void Awake()
    {
        _instance = this;
        materialPropertyBlock = new MaterialPropertyBlock();
    }
}
