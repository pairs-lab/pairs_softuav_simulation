# Khảo sát va tường: soft và rigid drone

Mô hình cơ học rút gọn, bộ điều khiển PD giống nhau; không phải policy RL hay digital twin đã hiệu chuẩn.

Khối lượng 0.97 kg; bước vật lý 0.0005 s; đáp ứng động cơ 0.04 s.
Ngưỡng gãy rigid giả định: mô-men tiếp xúc > 110.0 Nm hoặc lực > 2000.0 N tại một tay.

| Tốc độ đặt (m/s) | Loại | Biến dạng tay (°) | Nghiêng thân tối đa (°) | Tay tách | Phục hồi hover (s) | Chạm sàn |
|---:|:---|---:|---:|---:|---:|:---|
| 2 | soft | 10.1 | 12.7 | 0 | 1.007 | Không |
| 2 | rigid | 0.0 | 6.1 | 0 | 0.739 | Không |
| 2.5 | soft | 13.1 | 12.7 | 0 | 1.010 | Không |
| 2.5 | rigid | 0.0 | 8.0 | 0 | 0.668 | Không |
| 3 | soft | 16.6 | 12.9 | 0 | 1.011 | Không |
| 3 | rigid | 0.0 | 11.2 | 0 | 0.599 | Không |
| 3.5 | soft | 19.2 | 12.8 | 0 | 1.013 | Không |
| 3.5 | rigid | 0.0 | 14.7 | 0 | 0.605 | Không |
| 4 | soft | 22.0 | 12.7 | 0 | 1.014 | Không |
| 4 | rigid | 0.0 | 18.0 | 0 | 0.622 | Không |
| 4.5 | soft | 23.9 | 12.7 | 0 | 1.020 | Không |
| 4.5 | rigid | 0.0 | 21.2 | 0 | 0.634 | Không |
| 5 | soft | 26.2 | 12.7 | 0 | 1.023 | Không |
| 5 | rigid | 0.0 | 23.9 | 0 | 0.643 | Không |
| 5.5 | soft | 29.1 | 13.0 | 0 | 1.024 | Không |
| 5.5 | rigid | 0.0 | 180.0 | 2 | Không | Có |
| 6 | soft | 30.9 | 13.7 | 0 | 1.022 | Không |
| 6 | rigid | 0.0 | 180.0 | 2 | Không | Có |
| 6.5 | soft | 33.5 | 14.3 | 0 | 1.023 | Không |
| 6.5 | rigid | 0.0 | 179.9 | 2 | Không | Có |
| 7 | soft | 40.2 | 13.5 | 0 | 1.026 | Không |
| 7 | rigid | 0.0 | 180.0 | 2 | Không | Có |
| 7.6 | soft | 43.5 | 13.7 | 0 | 1.030 | Không |
| 7.6 | rigid | 0.0 | 180.0 | 2 | Không | Có |

Thời gian phục hồi tính từ tiếp xúc đầu tiên tới đầu khoảng ổn định kéo dài 0,3 s. Điều kiện: cách mục tiêu dưới 12 cm, tốc độ dưới 0,25 m/s, nghiêng dưới 8°, không tiếp xúc tường/sàn hoặc tách tay.

Xem summary.csv/results.json để đối chiếu vận tốc va chạm thực, lực đỉnh chưa lọc, gia tốc đỉnh chưa lọc, gia tốc giảm tốc trung bình 10 ms, xung lực và độ vượt giới hạn trượt.

**Giới hạn:** mô hình soft chưa tái hiện lợi thế về đỉnh lực tức thời của thực nghiệm. Các chặn hành trình trượt có thể tạo xung lực rất ngắn; độ vượt giới hạn được ghi riêng. Không dùng kết quả này làm ngưỡng an toàn vật liệu hoặc khẳng định HoLoArm thật sẽ sống sót khi đâm tường 8 m/s. Soft không có mô hình phá hủy vật liệu; rigid chỉ gãy theo các ngưỡng giả định ở trên.

[Bài báo HoLoArm](https://arxiv.org/html/2605.25790v1) và [video tham chiếu](https://www.youtube.com/watch?v=hmfgFP9XoHQ) phân biệt phục hồi khi bay với thử thả rơi 3 m. Kết quả tốc độ cao trong thử thả rơi không xác định trực tiếp ngưỡng gãy khi bay đâm tường.

Chi tiết phương pháp: ../../holoarm/WALL_COLLISION_STUDY.md
