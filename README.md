# legged_wheeled

## urdf

### `base_link`

![image-20220919212510659](README.assets/image-20220919212510659.png)

底盘+四个yaw电机

### left_front

#### `left_front_abad`

- parent: `base_link`
- child: `left_front_abad`
- type: `revolute`
- limit
  - lower: -3.14
  - upper: 3.14
  - effort: 10
  - velocity: 275

![image-20220919212632064](README.assets/image-20220919212632064.png)

#### `left_front_hip`

- parent: `left_front_abad`
- child: `left_front_hip`
- type: `revolute`
- limit
  - lower: -3.14
  - upper: 3.14
  - effort: 10
  - velocity: 275

![image-20220919212703200](README.assets/image-20220919212703200.png)

#### `left_front_knee`

- parent: `left_front_hip`
- child: `left_front_knee`
- type: `revolute`
- limit
  - lower: -3.14
  - upper: 3.14
  - effort: 10
  - velocity: 275

![image-20220919212735303](README.assets/image-20220919212735303.png)

#### `left_front_wheel`

- parent: `left_front_knee`
- child: `left_front_wheel`
- type: `revolute`
- limit
  - lower: -10000
  - upper: 10000
  - effort: 10
  - velocity: 400

![image-20220919212756792](README.assets/image-20220919212756792.png)



All other three legs are the same



