import os

# Height definitions per piece type (in meters)
PIECE_HEIGHTS = {
    "pawn": 0.053,
    "rook": 0.070,
    "knight": 0.070,
    "bishop": 0.070,
    "king": 0.075,
    "queen": 0.080
}

BOX_X = 0.025  # 2.5 cm base length
BOX_Y = 0.025  # 2.5 cm base width

WHITE_MATERIAL = """<material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/White</name>
          </script>
          <ambient>0.9 0.9 0.9 1</ambient>
          <diffuse>0.9 0.9 0.9 1</diffuse>
        </material>"""

BLACK_MATERIAL = """<material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
          <specular>0.9 0.9 0.9 1</specular>
          <shininess>100</shininess>
        </material>"""

def create_sdf(piece_type, color_prefix):
    is_white = (color_prefix == "w")
    folder_name = f"{color_prefix}_{piece_type}"
    model_name = folder_name if is_white else f"grey_{piece_type}"
    material = WHITE_MATERIAL if is_white else BLACK_MATERIAL
    
    height = PIECE_HEIGHTS[piece_type]
    # Center of collision box is half the height above bottom origin
    box_z_center = height / 2.0

    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>0.025</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.0001</iyy>
          <iyz>0.0</iyz>
          <izz>0.0001</izz>
        </inertia>
      </inertial>

      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://{folder_name}/{folder_name}.stl</uri>
            <scale>0.025 0.025 0.025</scale>
          </mesh>
        </geometry>
        {material}
      </visual>

      <!-- Collision Box scaled per piece height -->
      <collision name="collision">
        <pose>0 0 {box_z_center:.3f} 0 0 0</pose>
        <geometry>
          <box>
            <size>{BOX_X:.3f} {BOX_Y:.3f} {height:.3f}</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""

def generate():
    base_dir = os.path.dirname(os.path.abspath(__file__))

    for prefix in ["w", "b"]:
        for piece_type in PIECE_HEIGHTS.keys():
            folder = f"{prefix}_{piece_type}"
            folder_path = os.path.join(base_dir, folder)
            os.makedirs(folder_path, exist_ok=True)

            sdf_path = os.path.join(folder_path, "model.sdf")
            with open(sdf_path, "w") as f:
                f.write(create_sdf(piece_type, prefix))

            print(f"Generated {sdf_path} (Height: {PIECE_HEIGHTS[piece_type]}m)")

if __name__ == "__main__":
    generate()