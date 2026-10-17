# PAIRS Simulation — HoLoArm CAD, Windows

Integration note: these CAD tools now run directly from this repository.
Historical behavior scripts use `legacy_collision_study.py` and their original
parameters; the current CAD scripts use the frozen model described in
[holoarm/CAD_MODEL.md](holoarm/CAD_MODEL.md).
The MuJoCo 3.16.0 results below describe verification supplied with the import.
Repository integration checks also pass on the local MuJoCo 3.15.0 environment;
native Windows GUI verification has not been performed here.

## Chạy ngay

1. Giải nén toàn bộ gói vào thư mục mới, không giải nén đè môi trường .venv của bản trước.
2. Cần Python 64-bit có Python Launcher `py`; khuyên dùng Python 3.12.
3. Nhấp đúp **CHAY_HOLOARM_WINDOWS.bat**. Lần đầu cần Internet để tự cài requirements-windows.txt.
4. Chọn một trong ba nút:
   - **Bay qua khe hẹp:** MuJoCo tương tác, dùng chính lớp Flight của repository, model CAD thay thế.
   - **Va tường:** xem thí nghiệm tự chạy với cùng Trial của repository, phóng to và đổi góc nhìn được.
   - **Xuất video:** so sánh HoLoArm mềm với cùng CAD khóa arm; chờ cửa sổ dòng lệnh chạy xong rồi mở outputs.

Không cần Ubuntu, WSL, ROS, catkin, libX11 hoặc tải MuJoCo Menagerie cho ba nút này. FFmpeg cho xuất video được cài qua imageio-ffmpeg; không cần cài riêng. Máy vẫn cần trình điều khiển đồ họa có OpenGL.

## Chỉnh bề dày tường và xác nhận K/C

Hai ô bề dày đã có trong cửa sổ chạy. Hoặc sửa trực tiếp `holoarm/models/scene_config.json`, đóng rồi mở lại chương trình để lấy mặc định mới:

| Khóa trong JSON | Ý nghĩa | Mặc định |
|---|---|---:|
| impact_wall.thickness_m | Bề dày tường của bài va trực diện | 0,09 m |
| impact_wall.width_m / height_m | Bề rộng / cao của tường va | 2,4 / 2,5 m |
| passage.wall_thickness_x_m | Bề dày vật cản khe dọc hướng bay X | 0,50 m |
| passage.wall_depth_y_m | Chiều sâu khối tường theo Y, phía ngoài khe | 1,60 m |
| passage.guide_thickness_m | Bề dày tấm dẫn hướng xiên | 0,05 m |

Các số là kích thước đầy đủ bằng mét: nhập 0.05 để có 5 cm, không nhập 5 và không chia đôi. Code tự chuyển sang half-size của MuJoCo. Đổi bề dày tường va giữ mặt trước tại front_x_m; tâm tường dịch về sau. Đổi bề dày vật cản khe giữ nguyên bề rộng khe và tự cập nhật vị trí nối của hai tấm dẫn hướng. Bề rộng khe vẫn nhập ở ô riêng.

Bề dày là thông số hình học, không tự đổi độ cứng/ma sát tiếp xúc. Vì thế tường va chỉ đổi mặt sau sẽ không làm lực va đập ban đầu khác đi khi mặt trước và các điều kiện khác giữ nguyên. Tường khe dày hơn có thể thay đổi quãng đường tiếp xúc khi đi qua.

K và C đã có hiệu lực ở bản trước; bản cập nhật này chỉ tách nơi khai báo cho rõ ràng, không nhận dạng lại hay thay giá trị:

- `holoarm/models/holoarm_cad/stiffness.json`: bộ K phi tuyến đã chốt, được đọc trực tiếp bởi restoring().
- `holoarm/models/holoarm_cad/rheology.json`: nguyên bộ c0, km, tau, s v2, giữ nguyên byte từ gói trước; không chỉ thay một số C vào lò xo tuyến tính cũ.
- `holoarm/models/wall_collision_config.json`: đã bỏ các K/C tuyến tính minh họa của demo cũ. Kết quả JSON lúc chạy ghi cả bộ K và rheology đang dùng.

K ở lân cận zero tương ứng khoảng 8,694 N/mm dọc trục; 1,8694 N m/rad cho alpha; 2,3895 N m/rad cho beta. Luật phi tuyến và moment bão hòa nằm trong stiffness.json. c0 v2: x=708,1449365 N s/m; alpha=0,0063494793; beta Up=0,0082418352; beta Down=0,0098682942 N m s/rad. Các c0 này phải đi cùng nhánh nhớ và hệ số unloading đã có trong rheology.json. Việc chọn preset beta vẫn cố định cho từng bài như bản trước.

Ví dụ chạy bằng dòng lệnh:

```text
python holoarm/cad_wall_viewer.py --speed .6 --wall-thickness .05
python holoarm/holoarm_narrow_passage_viewer.py --gap .46 --passage-thickness .10
```

## Phím điều khiển

Khe hẹp: bấm vào cửa sổ MuJoCo để lấy focus. Giữ I/K hoặc mũi tên lên/xuống để bay theo world X; J/L hoặc trái/phải theo world Y. U/O lên/xuống, Q/E xoay yaw. Thả phím để hãm và giữ vị trí; H giữ vị trí; R reset; Space dừng/tiếp tục; F đổi camera; Esc đóng. Chuột để xoay và zoom ở camera overview. Nếu chuyển sang ứng dụng khác, lệnh bay được nhả để drone hãm.

Va tường: chạy chậm 0,25 lần để quan sát. Space dừng/tiếp tục, F góc nhìn, S lưu CSV/JSON, R đóng rồi mở lại bài từ đầu. Hết thời lượng tự dừng và lưu; có thể R để lặp lại. File nằm trong outputs/cad_wall_*.

Thông số mặc định: khe thông thủy 0,46 m; tốc độ 0,6 m/s. GUI cho chọn 0,3 / 0,6 / 0,9 m/s hoặc nhập trong 0,1–0,9. Đây là hai giao thức khác nhau: bài khe dùng điều khiển vận tốc liên tục; bài tường chuẩn bị vận tốc ban đầu theo giao thức Trial của repository, không dùng cách tiếp cận của gói MuJoCo trước. Kết quả có thể khác dù K, C không đổi.

## Những phần đã thay

- Hình học dựng từ **Assem1_Drone_final_design1.STEP** đã được người dùng cung cấp và duyệt trước: 37 mesh OBJ, 20 skin cho joint/ligament. Không dùng artwork soft_holoarm_visual.xml hay holoarm_physics.xml để tạo model chạy hiện hành.
- Cơ cấu giữ đúng thứ tự **trượt shaft → xoay alpha → xoay beta**. x dương là nén dọc trục về thân. Shaft, coupling và distal chuyển động theo chuỗi đã thống nhất; skin chỉ hiển thị, không tạo thêm lực.
- Đánh số: 1 trước-phải, 3 trước-trái, 2 sau-trái, 4 sau-phải. Ánh xạ nội bộ repository: fr=1, fl=3, rl=2, rr=4. Không xoay thêm CAD lần nữa.
- Khối lượng 0,970 kg, phần động mỗi arm 0,047 kg có chứa distal 0,035 kg, giữ như gói trước. Mesh và skin không cộng thêm khối lượng.
- K phi tuyến, C và toàn bộ bộ nhớ Maxwell v2 giữ nguyên, xem models/holoarm_cad/rheology.json và frozen_manifest.json. Các K/C tuyến tính của config demo cũ không được nạp vào cơ học CAD.
- Lực cản được khai báo ở DOF và MuJoCo tích phân ngầm một lần; lực đàn hồi phi tuyến và nhánh nhớ được áp dụng một lần trong cad_model.py. Không cộng lực ligament lần nữa.
- Motor dùng bốn actuator ở đúng rotor sites di động, có torque yaw theo dấu spin của HoLoArm. Giữ bộ điều khiển Flight/Trial của repository, cập nhật phân bổ lực đẩy theo cấu hình hiện thời.
- Bàn phím Windows dùng Win32 để đọc phím giữ khi cửa sổ MuJoCo của tiến trình đang foreground. Không dùng X11 hoặc hook bàn phím toàn cục. Nhánh Linux giữ backend X11 cũ.
- So sánh rigid dùng **cùng CAD, khối lượng và rotor layout**, khóa arm bằng weld làm đối chứng giả định. Không phải model Skydio X2 và không phải thí nghiệm gãy PLA. Mặc định không gãy; tùy chọn CLI `--assumed-fracture` chỉ dành cho khảo sát ngưỡng giả định cũ.

MuJoCo vẫn cần MJCF/XML và mesh để biểu diễn CAD. Chỉ mở XML bằng viewer chuẩn sẽ chưa chạy luật lực phi tuyến và bộ nhớ vật liệu v2; cần chạy qua các script/BAT ở trên.

## Tham số giữ nguyên và các giả thiết còn lại

Luật tải:

- Fx = 25,56 [1 − exp(−x/0,002940)] N cho x >= 0, với x tính m; nhánh kéo nhỏ tuyến tính như gói trước.
- M_alpha = 0,3830 tanh(1,8694 alpha / 0,3830) N m.
- M_beta = Ms tanh(2,3895 beta / Ms), Ms=0,3205 cho beta>=0, Ms=0,5609 cho beta<0.

Các c0/km/tau/s trong rheology.json được sao chép nguyên byte từ gói đã chốt. Beta dùng preset Up cho hover và va tường như bản toàn drone trước; chưa tự đổi preset theo dấu góc trong mỗi chu kỳ.

Vẫn giữ giả thiết preload nội cố định tại hình thái hover CAD danh định; không tuyên bố dự đoán võng tĩnh thật khi bay. COM/quán tính distal giữ từ bản một arm và quán tính thân là xấp xỉ trước đây, chưa phải phân bố vật liệu được nhận dạng đầy đủ từ CAD. Hình học CAD đúng không tự làm các giả thiết động lực học này thành đã xác nhận.

Guard–tường và guard–khe dùng tiếp xúc **chỉ theo pháp tuyến, trượt tự do theo tiếp tuyến**, giống bản bên phải đã thảo luận. Phản lực tiếp xúc do MuJoCo tính. Đây không phải hệ số ma sát tường đã đo. Không có lệnh đặt trước góc gập, ép hành trình hoặc dội ra theo hoạt ảnh. Bài tường dịch chuyển vị trí chuẩn bị và đặt vận tốc đầu trước khi bắt đầu thử, như repository gốc; không sửa qpos để tạo phản ứng sau va chạm.

Giới hạn x [-1, 8] mm và góc +/-0,70 rad là bảo vệ số từ gói trước, không phải hard stop thực nghiệm. Chạm giới hạn được ghi/cảnh báo. Mức 0,9 có thể vượt miền bench hoặc mất ổn định; không coi là dự báo khả năng sống sót thật. Kết quả hiện phục vụ quan sát hình thái và phát triển mô hình, chưa phải digital twin bay đã xác nhận hoặc policy RL đã train.

## Các file chính

- holoarm/cad_model.py: nạp CAD, gắn actuator và áp dụng mô hình vật liệu v2.
- holoarm/models/holoarm_cad/: mesh, CAD XML, flight_config.json, rheology.json nguyên gốc và hash để kiểm tra.
- holoarm/holoarm_narrow_passage_viewer.py: demo khe với CAD.
- holoarm/cad_wall_viewer.py: cửa sổ va tường tương tác.
- holoarm/collision_study.py: Trial và xuất video soft/locked CAD.
- UPSTREAM_INFO.json: repository/branch/commit nguồn. Bản này dựa nhánh ros1 vì có các demo mới; phần Windows chạy standalone, không dùng ROS. Chưa push lên GitHub.

Các script *behavior*, *restored*, *compression_then_lean*, *rigid_offset* và video cũ của repository là các khảo sát minh họa khác, không phải bản CAD đã chốt. Không dùng chúng làm lối chạy Windows mới; các transform cơ học của chúng bị từ chối khi tạo CAD để tránh âm thầm thay luật vật liệu. Model/artwork cũ còn trong mã nguồn để tham khảo, không được dùng ở ba nút mới. Skydio teleop là demo riêng, chưa thay bằng HoLoArm trong tác vụ này.

## Kiểm tra và giới hạn xác nhận

Đã kiểm tra headless trên Linux với MuJoCo 3.16.0, đọc đúng 37 mesh/20 skin, khối lượng, tọa độ rotor, dấu trượt, damping, hash các file giữ nguyên, điều khiển giữ/nhả phím và focus contract Windows giả lập. Khe 46 cm được đi qua bằng tiếp xúc động lực học; bài va 0,6 m/s soft/locked chạy và xuất video được. Xem CAD_VERIFICATION.md.

Chưa chạy trực tiếp GUI/Win32/OpenGL trên máy Windows thật; đây là phần cần kiểm tra khi bạn mở gói. Nếu lỗi, giữ cửa sổ dòng lệnh và gửi thông báo hiện ra.

Các kiểm tra liên quan bản CAD:

```text
python -m unittest discover -s holoarm -p test_cad_integration.py -v
python -m unittest discover -s holoarm -p test_narrow_passage_viewer.py -v
python -m unittest discover -s holoarm -p test_holoarm_ros_adapter.py -v
python holoarm/holoarm_narrow_passage_viewer.py --smoke-test
```

Không dùng toàn bộ bộ test hành vi demo cũ làm tiêu chuẩn cho model CAD; các test đó có giả thiết/artwork/tham số khác. Các test ROS adapter ở đây không cần cài ROS; chỉ kiểm tra adapter chia sẻ cùng model và Flight.
