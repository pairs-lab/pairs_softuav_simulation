# Kiểm tra bản tích hợp CAD

This is the verification report supplied with the CAD package. Its reference
artifacts are preserved in `outputs/cad_checks/`. Current model sources and
assumptions are documented in [holoarm/CAD_MODEL.md](holoarm/CAD_MODEL.md).

Môi trường tính toán: Linux, Python 3.12, MuJoCo 3.16.0. Chưa kiểm tra GUI/Win32 trên Windows thật.

## Các kiểm tra tự động

- 8 kiểm tra CAD/input: hình học, khối lượng, tọa độ motor và đánh số; C chỉ được áp dụng một lần; file tham số nguyên byte; luật K phi tuyến; cùng CAD cho rigid; mask tiếp xúc khe đúng; khởi tạo va chạm; đọc phím có điều kiện focus Windows qua API giả lập; đổi bề dày tường giữ nguyên mặt trước và quỹ đạo va; đổi bề dày vật cản khe giữ khe và nối đúng các tấm dẫn hướng.
- 7 kiểm tra viewer: giữ/nhả phím, dừng/reset, giới hạn điều khiển, hover, skin ligament thực chuyển động theo khớp và không vẽ trùng artwork cũ, các phím native không đổi kiểu hiển thị.
- 3 kiểm tra adapter dùng cùng Flight/model: không cần cài ROS; quỹ đạo và dữ liệu giống viewer, đi qua khe và giữ vị trí đích.

Đã chạy và pass các nhóm kiểm tra trên. Các test riêng của các behavior demo cũ không thuộc tiêu chuẩn nghiệm thu CAD này.

## Khe 46 cm, tốc độ giới hạn 0,6 m/s

Lệnh: `python holoarm/holoarm_narrow_passage_viewer.py --smoke-test`.

- Sai số hover ban đầu: khoảng 0,00000150 m.
- Điểm kết thúc: [3,65408; 0,00000473; 1,19945] m.
- Có 4017 bước có tiếp xúc.
- Góc uốn cực đại: 23,41 độ.
- Nghiêng thân cực đại: 17,67 độ.
- Độ cao thấp nhất: 1,13126 m.
- Tốc độ kết thúc: 0,01799 m/s.
- Cảnh báo solver: 0.

Đây là kết quả của mô hình với controller và tiếp xúc đang chọn, không phải số đo bay thật.

## Va tường 0,6 m/s

Giao thức Trial của repository: settle 0,8 s, chuẩn bị khoảng tiếp cận và đặt vận tốc ban đầu trước khi bắt đầu thử. Thời gian sau va dự kiến 1,5 s; tiếp xúc normal-only. Soft và đối chứng khóa arm dùng cùng CAD và không bật ngưỡng gãy.

- Tốc độ thực lúc chạm: khoảng 0,600 m/s ở cả hai bên.
- Soft: uốn cực đại khoảng 17,5 độ; nghiêng thân cực đại 25,3 độ; thỏa tiêu chí hover của demo sau khoảng 0,965 s từ tiếp xúc.
- Locked: uốn 0; nghiêng thân cực đại khoảng 4,6 độ; thỏa tiêu chí hover của demo sau khoảng 0,936 s.

Không suy ra ưu thế chung của soft từ một lượt này. Tiêu chí hồi phục do repository định nghĩa, không phải xác nhận thực nghiệm. File CSV/JSON và clip kiểm tra nằm trong outputs/cad_checks/.

## Bất biến cần giữ

37 mesh, 20 skin; tổng khối lượng 0,970 kg. Model mềm nq=19, nv=18, nu=4; có 4 slide và 8 hinge. Rigid đối chứng có năm free body kết nối bằng bốn weld và cùng mesh/khối lượng. Bộ rheology.json, flight_config.json, CAD XML và các mesh có hash trong models/holoarm_cad/frozen_manifest.json.

Không thay K/C để làm khớp kết quả demo. Không dùng giới hạn số hay kết quả không có cảnh báo solver làm bằng chứng mô hình đúng vật lý. Preload hover, quán tính được kế thừa và ma sát lý tưởng vẫn là giả thiết.

## Repository integration verification (2026-10-17)

The imported model was checked in the main repository with Python 3.12 and
MuJoCo 3.15.0, preserving the supplied geometry and parameter bytes.

- All 51 tests passed with `python -m unittest discover -s holoarm -p 'test_*.py'`.
  This includes current CAD/material/viewer/adapter tests and the separately
  preserved historical behavior tests.
- The headless passage test crossed the 46 cm gap: final X 3.65408 m, minimum
  height 1.13126 m, maximum bend 23.4067 degrees, final speed 0.01799 m/s,
  4017 contact steps and zero solver warnings.
- The 0.6 m/s wall CLI run measured 0.600 m/s incoming speed in both cases.
  Soft/locked recovery times were approximately 0.9652/0.9364 s with fracture
  disabled. These are observed model outcomes, not a required soft advantage.
- The numerical audit at timesteps 0.0004 and 0.0002 s passed its sensitivity
  check. Relative peak-force changes were 1.686% soft and 0.0835% locked.
  Both cases recovered without floor contact, guard contact or solver warnings.
- An isolated copy in the catkin share layout loaded all 37 meshes and 20 skins
  and stepped through the adapter using the explicit installed scene path.
- After removal of the duplicate import directory, both CAD models built,
  Flight stepped and all 41 frozen checksums still matched.

Native Windows input/GUI and full OpenGL video rendering were not exercised
in this integration environment. The supplied reference clip remains in
`outputs/cad_checks/`; existing user baseline outputs were not regenerated.
