# Unity deterministic physics

This is a modified version of [Unity DOTS Physics version 0.5.1-preview.2](https://docs.unity3d.com/Packages/com.unity.physics@0.5/manual/index.html), which supports cross-platform deterministic physics simulation by using [soft floats](https://github.com/Kimbatt/soft-float-starter-pack).

## Usage
The following packages must be installed:
- Burst
- Collections
- Entities
- Jobs

You'll need to use the `UnityS.Physics` and the `UnityS.Mathematics` namespaces instead of the usual `Unity.Physics` and `Unity.Mathematics`.

See [Unity Physics 0.5.1-preview.2 manual](https://docs.unity3d.com/Packages/com.unity.physics@0.5/manual/index.html) for documentation.

## Notes on determinism
For a deterministic physics simulation:
- All bodies must be created in the same order every time you create the simulation
- Bodies must only be modified in a system that is updated in FixedStepSimulationSystemGroup
- You must not use the result of floating point operations as input for the physics engine
- Be careful when using hashmaps / hashsets, because iterating over them may be non-deterministic (it depends on the implementation).
- Check how to use soft floats [here](https://github.com/Kimbatt/soft-float-starter-pack#how-to-use).
- Make sure that your code is also deterministic.

## Example
See the ExampleScene scene for an example. After running it, your simulation should look exactly like this:
![](deterministic.gif)

## License
`Unity.Physics`, `Unity.Mathematics`, and `Unity.Transforms` are licensed under the [Unity Companion License](https://unity3d.com/legal/licenses/Unity_Companion_License).

`Soft floats` is licensed under the MIT License. See [https://github.com/Kimbatt/soft-float-starter-pack](https://github.com/Kimbatt/soft-float-starter-pack) for more information.
