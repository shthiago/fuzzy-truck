from fuzzy_truck import Driver

if __name__ == '__main__':
    driver = Driver('127.0.0.1', 4321)

    for i in range(50):
        driver.play()