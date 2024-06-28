# RobotWorld compileren en gebruiken

Deze code is gecompileerd en gebruikt met: 

- Ubuntu 22.04.4 LTS (jammy).
- Boost versie 1.74.0
- wxWidgets versie 3.2.2.1

## Compilatie-instructies

De compilatie-instructies zijn hetzelfde als bij de originele versie van RobotWorld. Voor instructies over compileren op Windows of Linux moet op de onderstaande link worden geklikt.
https://bitbucket.aimsites.nl/projects/ESDD/repos/robotworld/browse/readme.md

## Gebruik van de applicatie

### Opstarten van de applicatie

Als je de applicatie hebt gecompileerd, kun je deze runnen met:
```bash
./robotworld
```

Als je twee instanties van de applicatie met elkaar wilt laten communiceren, moeten de local_port en de remote_port ook worden ingevuld:
```bash
./robotworld -local_port=54321 -remote_port=12345
```
Uiteraard mag je zelf bepalen welke getallen de local_port en de remote_port zijn. De local_port van de ene applicatie moet wel de remote_port van de andere zijn (en andersom).

Als je twee applicaties op aparte apparaten wilt laten communiceren, moet ook het ip-adres van het andere apparaat worden ingevuld.
```bash
./robotworld -local_port=54321 -remote_port=12345 -remote_ip=192.168.1.1
```

### Besturing RobotWorld

Deze instructies gaan ervanuit dat je twee applicaties hebt opgestart.

#### Uitvoeren scenario

Hieronder is te lezen hoe een scenario kan worden uitgevoerd. In dit voorbeeld is voor scenario 1 gekozen, maar de andere scenario's zijn natuurlijk ook mogelijk.
1. Klik bij de ene applicatie op 'Scenario 1.1' en bij de andere applicatie op 'Scenario 1.2' (of andersom).
2. Klik bij beide applicaties op 'Start listening' om de communicatie tussen de twee applicaties op te starten.
3. Als je bij één van de twee applicaties op 'Sync worlds' klikt, zal de opzet van scenario 1 nu op beide schermen te zien zijn.
4. Klik bij één van de twee applicaties op 'Start both robots', zodat beide robots gaan rijden. De robot van de applicatie waarmee je op 'Sync worlds' hebt geklikt zal dan automatisch om de andere robot heen rijden als ze bijna tegen elkaar aan botsen.

#### Nog een scenario uitvoeren

Als je al een scenario hebt gedaan en de applicatie(s) nog niet hebt afgesloten, kun je de vorige instructies nog eens volgen. Er zijn echter twee dingen waarop moet worden gelet.
- Stap 2 kan worden overgeslagen. Je hoeft niet nog een keer op 'Start listening' te klikken.
- Klik vooral niet op 'Unpopulate'. Dit haalt de robot wel uit de wereld, maar door een shared pointer vanuit de clientserver-sessie wordt de robot niet volledig uit de applicatie verwijderd, wat resulteert in undefined behavior. Mocht je toch op 'Unpopulate' hebben geklikt, start dan de applicatie opnieuw op.
