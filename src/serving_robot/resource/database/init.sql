
CREATE SCHEMA Restaurant;

CREATE TABLE Restaurant.orders (
    order_id INT AUTO_INCREMENT PRIMARY KEY,
    total_price DECIMAL(10, 2) NOT NULL,
    table_number INT NOT NULL,
    created_date DATETIME DEFAULT CURRENT_TIMESTAMP,
    updated_date DATETIME DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
);

CREATE TABLE Restaurant.products (
    product_id INT AUTO_INCREMENT PRIMARY KEY,   -- 제품 ID (프라이머리 키)
    name VARCHAR(255) NOT NULL,           -- 제품 이름
    price DECIMAL(10, 2) NOT NULL         -- 제품 가격
);
CREATE TABLE Restaurant.orders_product (
    order_product_id INT AUTO_INCREMENT PRIMARY KEY,
    order_id INT NOT NULL,
    product_id INT NOT NULL,
    quantity INT NOT NULL,
    price DECIMAL(10, 2),
    FOREIGN KEY (order_id) REFERENCES orders(order_id),
    FOREIGN KEY (product_id) REFERENCES products(product_id) -- 상품 테이블이 있다고 가정
);
USE Restaurant;

INSERT INTO products (product_id, name, price)
VALUES
(1, '짜장면', 6000),
(2, '간짜장', 7000),
(3, '쟁반짜장', 8000),
(4, '짬뽕', 8000),
(5, '짬뽕밥', 7000),
(6, '짜장밥', 7000),
(7, '탕수육', 15000),
(8, '깐풍기', 15000),
(9, '군만두', 8000),
(10, '팔보채', 15000),
(11, '고추잡채', 15000),
(12, '꽃 빵', 6000),
(13, '사이다', 2000),
(14, '콜라', 2000),
(15, '환타', 2000),
(16, '소주', 5000),
(17, '맥주', 5000),
(18, '고량주', 10000);
-- 오늘 날짜 데이터 20개 삽입

INSERT INTO orders (order_id, total_price, table_number, created_date, updated_date)
VALUES
(7, 10000.00, 1, NOW(), NOW()),
(8, 13000.00, 2, NOW(), NOW()),
(9, 4000.00, 3, NOW(), NOW());
-- 어제 날짜 데이터 20개 삽입
INSERT INTO orders (order_id, total_price, table_number, created_date, updated_date)
VALUES
(1, 12000.00, 1, DATE_SUB(NOW(), INTERVAL 1 DAY), DATE_SUB(NOW(), INTERVAL 1 DAY)),
(2, 14000.00, 2, DATE_SUB(NOW(), INTERVAL 1 DAY), DATE_SUB(NOW(), INTERVAL 1 DAY)),
(3, 21000.00, 4, DATE_SUB(NOW(), INTERVAL 1 DAY), DATE_SUB(NOW(), INTERVAL 1 DAY));
-- 2일 전 날짜 데이터 20개 삽입
INSERT INTO orders (order_id, total_price, table_number, created_date, updated_date)
VALUES
(4,30000.00, 5, DATE_SUB(NOW(), INTERVAL 2 DAY), DATE_SUB(NOW(), INTERVAL 2 DAY)),
(5, 20000.00, 6, DATE_SUB(NOW(), INTERVAL 2 DAY), DATE_SUB(NOW(), INTERVAL 2 DAY)),
(6, 25000.00, 8, DATE_SUB(NOW(), INTERVAL 2 DAY), DATE_SUB(NOW(), INTERVAL 2 DAY));
INSERT INTO orders_product (order_product_id, order_id, product_id, quantity, price)
VALUES
(1, 1, 1, 1, 6000.00),   -- 짜장면 (6,000원)
(2, 1, 1, 1, 6000.00),   -- 짜장면 (6,000원)
(3, 2, 1, 3, 6000.00),   -- 짜장면 (6,000원)
(4, 2, 4, 1, 8000.00),   -- 짬뽕 (8,000원)
(5, 3, 1, 2, 6000.00),   -- 짜장면 (6,000원)
(6, 3, 7, 1, 15000.00),  -- 탕수육 (15,000원)
(7, 4, 7, 3, 15000.00),  -- 탕수육 (15,000원)
(8, 4, 8, 1, 15000.00),  -- 깐풍기 (15,000원)
(9, 5, 8, 1, 15000.00),  -- 깐풍기 (15,000원)
(10, 5, 16, 1, 5000.00), -- 소주 (5,000원)
(11, 6, 8, 1, 15000.00), -- 깐풍기 (15,000원)
(12, 6, 18, 3, 10000.00),-- 고량주 (10,000원)
(13, 7, 16, 1, 5000.00), -- 소주 (5,000원)
(14, 7, 17, 2, 5000.00), -- 맥주 (5,000원)
(15, 8, 1, 1, 6000.00),  -- 짜장면 (6,000원)
(16, 8, 2, 1, 7000.00),  -- 간짜장 (7,000원)
(17, 9, 14, 3, 2000.00), -- 콜라 (2,000원)
(18, 9, 15, 1, 2000.00); -- 환타 (2,000원)
