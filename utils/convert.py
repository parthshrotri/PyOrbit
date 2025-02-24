import datetime

def AU_to_meters(AU):
    return AU*149597870700

def meters_to_AU(meters):
    return meters/149597870700

def km_to_AU(km):
    return km/149597870.700

def AU_to_km(au):
    return au*149597870.700

def convertSecToDays(seconds):
    return seconds/(24*60*60)

def daysSinceJ2000(date, sec_since_date=0):
    j2000 = datetime.datetime(2000, 1, 1, 12, 0, 0)
    since_j2000 = (date - j2000).total_seconds() + sec_since_date
    return convertSecToDays(since_j2000)