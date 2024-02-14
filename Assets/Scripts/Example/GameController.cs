using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using UnityEngine;
using UnityS.Mathematics;
using UnityS.Physics;
using UnityS.Physics.Systems;
using UnityS.Transforms;

public partial class GameController : SystemBase
{
    public static GameController Instance;

    // store entity-gameobject pairs for rendering (not really efficient to use gameobjects here)
    private readonly Dictionary<Entity, GameObject> objects = new Dictionary<Entity, GameObject>(32);

    private MaterialPropertyBlock matPropBlock;

    protected override void OnStartRunning()
    {
        base.OnCreate();
        Instance = this;
        Physics.simulationMode = SimulationMode.Script;
        matPropBlock = new MaterialPropertyBlock();

        // setup physics parameters
        World.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>().Timestep = (float)(sfloat.One / (sfloat)60.0f);
        Entity physicsStep = EntityManager.CreateEntity(typeof(PhysicsStep));
        PhysicsStep physicsStepParams = PhysicsStep.Default;
        physicsStepParams.SolverStabilizationHeuristicSettings = new Solver.StabilizationHeuristicSettings
        {
            EnableSolverStabilization = true,
            EnableFrictionVelocities = true,
            InertiaScalingFactor = (sfloat)0.0f,
            VelocityClippingFactor = (sfloat)1.0f
        };

        physicsStepParams.SolverIterationCount = 3;
        physicsStepParams.MultiThreaded = 1;
        physicsStepParams.Gravity = new float3(sfloat.Zero, (sfloat)(-60.0f), sfloat.Zero);
        EntityManager.SetComponentData(physicsStep, physicsStepParams);

        UnityS.Physics.Material material = UnityS.Physics.Material.Default;
        material.Friction = sfloat.One;

        PhysicsParams physicsParamsStatic = PhysicsParams.Default;
        physicsParamsStatic.isDynamic = false;

        PhysicsParams physicsParamsDynamic = PhysicsParams.Default;
        physicsParamsDynamic.isDynamic = true;

        CreateBox(new float3(sfloat.Zero, sfloat.Zero, sfloat.Zero),
            new float3((sfloat)500.0f, (sfloat)2.0f, (sfloat)500.0f), quaternion.identity, material,
            physicsParamsStatic);

        sfloat radius = (sfloat)10.0f;
        int count = 7;
        sfloat sfcount = (sfloat)count;

        int layers = 4;

        sfloat size = (sfloat)6.0f;
        sfloat anglePerLayer = (sfloat)0.25f;

        // set up stacked boxes
        for (int l = 0; l < layers; l++)
        {
            sfloat offsetY = sfloat.One + ((sfloat)l + (sfloat)0.5f) * size;
            sfloat angleOffset = anglePerLayer * (sfloat)l;

            for (int i = 0; i < count; i++)
            {
                sfloat t = (sfloat)i / sfcount * math.PI * (sfloat)2.0f + angleOffset;
                math.sincos(t, out sfloat sin, out sfloat cos);
                float3 pos = new float3(radius * cos, offsetY, radius * sin);

                CreateBox(pos, new float3(size, size, size),
                    quaternion.AxisAngle(new float3(sfloat.Zero, sfloat.One, sfloat.Zero), -t),
                    material, physicsParamsDynamic);
            }
        }
    }

    protected override void OnUpdate()
    {
        Entities.ForEach((ref Entity e, ref Translation t, ref Rotation r, ref PhysicsVelocity _vel) =>
        {
            // update object transforms, based on ECS data
            if (objects.TryGetValue(e, out GameObject obj))
            {
                obj.transform.localPosition = (Vector3)t.Value;
                obj.transform.localRotation = (Quaternion)r.Value;
            }
        }).WithoutBurst().Run();
    }

    private PCG rand = default;
    private bool randInitialized = false;

    private Color RandomColor()
    {
        if (!randInitialized)
        {
            rand = new PCG(0, (ulong)ResourceManager.Instance.seed);
            randInitialized = true;
        }

        return Color.HSVToRGB(
            (float)rand.SFloatInclusive(sfloat.Zero, sfloat.One),
            (float)rand.SFloatInclusive(sfloat.Zero, sfloat.One),
            (float)rand.SFloatInclusive((sfloat)0.5f, sfloat.One)
        );
    }

    private Dictionary<(sfloat radius, UnityS.Physics.Material material), BlobAssetReference<UnityS.Physics.Collider>>
        ballColliders =
            new Dictionary<(sfloat radius, UnityS.Physics.Material material),
                BlobAssetReference<UnityS.Physics.Collider>>();

    public void CreateBall(float3 position, sfloat radius, UnityS.Physics.Material material,
        PhysicsParams physicsParams)
    {
        if (!ballColliders.TryGetValue((radius, material), out BlobAssetReference<UnityS.Physics.Collider> collider))
        {
            collider = UnityS.Physics.SphereCollider.Create(
                new SphereGeometry { Center = float3.zero, Radius = radius }, CollisionFilter.Default, material);

            ballColliders.Add((radius, material), collider);
        }

        Entity entity = CreateEntity(position, quaternion.identity, collider, physicsParams);

        GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Sphere);

        obj.transform.localScale = new Vector3((float)radius, (float)radius, (float)radius) * 2;
        obj.transform.localPosition = (Vector3)position;

        MeshRenderer renderer = obj.GetComponent<MeshRenderer>();
        renderer.material = (ResourceManager.Instance.defaultMaterial);
        renderer.GetPropertyBlock(matPropBlock);
        matPropBlock.SetColor("_Color", RandomColor());
        renderer.SetPropertyBlock(matPropBlock);

        objects.Add(entity, obj);
    }

    private Dictionary<(float3 size, UnityS.Physics.Material material), BlobAssetReference<UnityS.Physics.Collider>>
        boxColliders =
            new Dictionary<(float3 size, UnityS.Physics.Material material),
                BlobAssetReference<UnityS.Physics.Collider>>();

    public void CreateBox(float3 position, float3 size, quaternion rotation, UnityS.Physics.Material material,
        PhysicsParams physicsParams)
    {
        if (!boxColliders.TryGetValue((size, material), out BlobAssetReference<UnityS.Physics.Collider> collider))
        {
            collider = UnityS.Physics.BoxCollider.Create(
                new BoxGeometry
                {
                    Center = float3.zero,
                    Size = size,
                    Orientation = quaternion.identity
                }, CollisionFilter.Default, material
            );

            boxColliders.Add((size, material), collider);
        }

        Entity entity = CreateEntity(position, rotation, collider, physicsParams);

        GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Cube);

        obj.transform.localScale = (Vector3)size;
        obj.transform.localPosition = (Vector3)position;
        obj.transform.localRotation = (Quaternion)rotation;

        MeshRenderer renderer = obj.GetComponent<MeshRenderer>();
        renderer.material = (ResourceManager.Instance.defaultMaterial);
        renderer.GetPropertyBlock(matPropBlock);
        matPropBlock.SetColor("_Color", RandomColor());
        renderer.SetPropertyBlock(matPropBlock);

        objects.Add(entity, obj);
    }

    public Entity CreateEntity(float3 position, quaternion rotation,
        BlobAssetReference<UnityS.Physics.Collider> collider,
        PhysicsParams physicsParams)
    {
        return CreatePhysicsBody(position, rotation, collider, physicsParams);
    }

    private readonly List<ComponentType> componentTypes = new List<ComponentType>(20);

    public unsafe Entity CreatePhysicsBody(float3 position, quaternion orientation,
        BlobAssetReference<UnityS.Physics.Collider> collider,
        PhysicsParams physicsParams)
    {
        componentTypes.Clear();

        componentTypes.Add(typeof(Translation));
        componentTypes.Add(typeof(Rotation));
        componentTypes.Add(typeof(LocalToWorld));
        componentTypes.Add(typeof(PhysicsCollider));
        componentTypes.Add(typeof(PhysicsCustomTags));

        if (physicsParams.isDynamic)
        {
            componentTypes.Add(typeof(PhysicsVelocity));
            componentTypes.Add(typeof(PhysicsMass));
            componentTypes.Add(typeof(PhysicsDamping));
        }

        Entity entity = EntityManager.CreateEntity(componentTypes.ToArray());

        EntityManager.SetComponentData(entity, new Translation { Value = position });
        EntityManager.SetComponentData(entity, new Rotation { Value = orientation });

        EntityManager.SetComponentData(entity, new PhysicsCollider { Value = collider });

        if (physicsParams.isDynamic)
        {
            UnityS.Physics.Collider* colliderPtr = (UnityS.Physics.Collider*)collider.GetUnsafePtr();
            EntityManager.SetComponentData(entity,
                PhysicsMass.CreateDynamic(colliderPtr->MassProperties, physicsParams.mass));
            // Calculate the angular velocity in local space from rotation and world angular velocity
            float3 angularVelocityLocal =
                math.mul(math.inverse(colliderPtr->MassProperties.MassDistribution.Transform.rot),
                    physicsParams.startingAngularVelocity);
            EntityManager.SetComponentData(entity, new PhysicsVelocity()
            {
                Linear = physicsParams.startingLinearVelocity,
                Angular = angularVelocityLocal
            });
            EntityManager.SetComponentData(entity, new PhysicsDamping()
            {
                Linear = physicsParams.linearDamping,
                Angular = physicsParams.angularDamping
            });
        }

        return entity;
    }
}

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateBefore(typeof(BuildPhysicsWorld))]
public partial class PhysicsController : SystemBase
{
    private int frame = 0;

    private void ShootBall(sfloat mass, sfloat radius, float3 startingPos, float3 startingVelocity)
    {
        UnityS.Physics.Material material = UnityS.Physics.Material.Default;
        material.Friction = (sfloat)0.25f;
        material.Restitution = (sfloat)0.25f;

        PhysicsParams physicsParams = PhysicsParams.Default;
        physicsParams.isDynamic = true;
        physicsParams.startingLinearVelocity = startingVelocity;
        physicsParams.mass = mass;
        physicsParams.angularDamping = (sfloat)0.9f;

        GameController.Instance.CreateBall(startingPos, radius, material, physicsParams);
    }

    private void ShootBox(sfloat mass, float3 size, float3 startingPos, float3 startingVelocity)
    {
        UnityS.Physics.Material material = UnityS.Physics.Material.Default;
        material.Friction = (sfloat)0.25f;

        PhysicsParams physicsParams = PhysicsParams.Default;
        physicsParams.isDynamic = true;
        physicsParams.startingLinearVelocity = startingVelocity;
        physicsParams.mass = mass;

        GameController.Instance.CreateBox(startingPos,
            size, quaternion.identity, material, physicsParams);
    }

    protected override void OnUpdate()
    {
        if (frame >= 60 && frame % 10 == 0 && frame < 500)
        {
            ShootBox((sfloat)0.25f, (sfloat)7.0f, new float3((sfloat)50.0f, (sfloat)20.0f, (sfloat)(-50.0f)),
                new float3((sfloat)(-100.0f), (sfloat)1.0f, (sfloat)100.0f));
        }

        if (frame >= 100 && frame % 30 == 0 && frame < 900)
        {
            ShootBall((sfloat)0.25f, (sfloat)5.0f, new float3((sfloat)0.0f, (sfloat)200.0f, (sfloat)(0.0f)),
                float3.zero);
        }

        if (frame == 720)
        {
            ShootBall((sfloat)20.0f, (sfloat)12.0f, new float3((sfloat)50.0f, (sfloat)20.0f, (sfloat)(-50.0f)),
                new float3((sfloat)(-100.0f), (sfloat)1.0f, (sfloat)100.0f));
        }

        ++frame;
    }
}

public struct PhysicsParams
{
    public float3 startingLinearVelocity;
    public float3 startingAngularVelocity;
    public sfloat mass;
    public bool isDynamic;
    public sfloat linearDamping;
    public sfloat angularDamping;

    public static PhysicsParams Default => new PhysicsParams
    {
        startingLinearVelocity = float3.zero,
        startingAngularVelocity = float3.zero,
        mass = sfloat.One,
        isDynamic = true,
        linearDamping = sfloat.FromRaw(0x3c23d70a),
        angularDamping = sfloat.FromRaw(0x3d4ccccd)
    };
}