# Khảo sát va tường: soft và rigid drone

Mô hình cơ học rút gọn, bộ điều khiển PD giống nhau; không phải policy RL hay digital twin đã hiệu chuẩn.

Khối lượng 0.97 kg; bước vật lý 0.0004 s; đáp ứng động cơ 0.04 s.
Ngưỡng gãy rigid giả định: mô-men tiếp xúc > 110.0 Nm hoặc lực > 2000.0 N tại một tay.

| Tốc độ đặt (m/s) | Loại | Biến dạng tay (°) | Nghiêng thân tối đa (°) | Tay tách | Phục hồi hover (s) | Chạm sàn |
|---:|:---|---:|---:|---:|---:|:---|
| 0.6 | soft | 17.5 | 25.3 | 0 | 0.965 | Không |
| 0.6 | rigid | 0.0 | 4.6 | 0 | 0.936 | Không |

Thời gian phục hồi tính từ tiếp xúc đầu tiên tới đầu khoảng ổn định kéo dài 0,3 s. Điều kiện: cách mục tiêu dưới 12 cm, tốc độ dưới 0,25 m/s, nghiêng dưới 8°, không tiếp xúc tường/sàn hoặc tách tay.

Xem summary.csv/results.json để đối chiếu vận tốc va chạm thực, lực đỉnh chưa lọc, gia tốc đỉnh chưa lọc, gia tốc giảm tốc trung bình 10 ms, xung lực và độ vượt giới hạn trượt.

**Giới hạn:** mô hình soft chưa tái hiện lợi thế về đỉnh lực tức thời của thực nghiệm. Các chặn hành trình trượt có thể tạo xung lực rất ngắn; độ vượt giới hạn được ghi riêng. Không dùng kết quả này làm ngưỡng an toàn vật liệu hoặc khẳng định HoLoArm thật sẽ sống sót khi đâm tường 8 m/s. Soft không có mô hình phá hủy vật liệu; rigid chỉ gãy theo các ngưỡng giả định ở trên.

[Bài báo HoLoArm](https://arxiv.org/html/2605.25790v1) và [video tham chiếu](https://www.youtube.com/watch?v=hmfgFP9XoHQ) phân biệt phục hồi khi bay với thử thả rơi 3 m. Kết quả tốc độ cao trong thử thả rơi không xác định trực tiếp ngưỡng gãy khi bay đâm tường.

Chi tiết phương pháp: ../../holoarm/WALL_COLLISION_STUDY.md
