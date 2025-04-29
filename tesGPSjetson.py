import gps

session = gps.gps(mode=gps.WATCH_ENABLE)

while True:
    report = session.next()
    if report['class'] == 'TPV':
        if hasattr(report, 'lat') and hasattr(report, 'lon'):
            print(f"Latitude: {report.lat}, Longitude: {report.lon}")
