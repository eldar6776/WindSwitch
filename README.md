# WindSwitch

**Zaštitni uređaj za vanjske rolo roletne od jakog vjetra**

---

## Opis

WindSwitch je elektronski uređaj namijenjen za zaštitu rolo roletni na stambenim objektima od oštećenja uzrokovanih jakim vjetrom. Uređaj se napaja sa 220V i na ulazu koristi optocoupler izolovani impulsni ulaz iz anemometra (senzora vjetra). Kada vjetar premaši zadani prag brzine na definisano vrijeme, WindSwitch automatski aktivira podizače roletni preko četiri izolovana triaca, čime štiti roletne od oštećenja.

---

## Ključne funkcije

- **Optocoupler ulaz:** Sigurna galvanska izolacija za impulse sa anemometra.
- **Dva potenciometra na prednjem panelu:**
  - **Prvi potenciometar:** Podešavanje praga okidanja (broj impulsa = brzina vjetra).
  - **Drugi potenciometar:** Podešavanje minimalnog vremena aktivnog stanja (koliko dugo roletne ostaju podignute nakon aktivacije).
- **Logika aktivacije:** Uređaj aktivira izlaz (triaci) samo ako je brzina vjetra viša od praga više od 5 sekundi.
- **Automatski restart vremena:** Svaki ponovni prelazak praga okidanja resetuje aktivno vrijeme.
- **4 izolovana izlaza:** Kontrola podizača roletni putem triaca.
- **LED indikatori:** Signalizacija stanja uređaja.

---

## Tehničke karakteristike

- **Napajanje:** 220V AC
- **Ulaz:** Optocoupler impulsni ulaz sa anemometra
- **Izlazi:** 4 x triac, izolovani, za aktivaciju podizača roletni
- **Mikrokontroler:** STM32 (C, HAL biblioteka)
- **Potenciometri:** Za prag vjetra i vrijeme aktivnog stanja
- **LED indikacija:** Status i aktivacija

---

## Povezivanje

1. **Napajanje:** Priključiti uređaj na 220V.
2. **Anemometar:** Spojiti impulsni izlaz anemometra na optocoupler ulaz WindSwitch-a.
3. **Izlazi:** Povezati podizače roletni na četiri izlazna terminala (triaci).
4. **Potenciometri:** Podesiti prag vjetra i trajanje aktivnog stanja prema potrebama objekta.
5. **LED:** Pratiti indikaciju stanja (status, aktivacija, greške).

---

## Logika rada

- **Mjerenje vjetra:** Uređaj broji impulse sa anemometra i koristi mapiranje ADC vrijednosti potenciometra za određivanje praga.
- **Aktivacija:** Ako je brzina vjetra veća od praga kontinuirano više od 5 sekundi, aktiviraju se izlazi (podizači roletni).
- **Trajanje aktivnog stanja:** Drugi potenciometar određuje minimalno vrijeme koliko roletne ostaju podignute.
- **Reset vremena:** Svaki prelazak praga tokom aktivnog stanja resetuje timer za podignute roletne.
- **Bez prelaska praga:** Roletne ostaju u standardnom položaju.

---

## Instalacija i korištenje

1. Postavite WindSwitch na sigurno mjesto unutar objekta.
2. Spojite napajanje, impulsni ulaz sa anemometra i izlaze prema šemi povezivanja.
3. Podesite prag vjetra i vrijeme aktivnog stanja.
4. Pokrenite uređaj, LED indikatori signaliziraju status rada.
5. Pratite rad uređaja – kad vjetar premaši prag, roletne se automatski podižu na zadano vrijeme.

---

## Tehnologije

- **C (STM32 HAL)**
- **Optocoupler, triac izlazi**
- **Analogni potenciometri za podešavanje**
- **LED indikacija**
- **Sigurnosni watchdog timer**

---

## Sigurnosne napomene

- Uređaj radi sa visokonaponskim izlazima – instalaciju vrši kvalificirani električar!
- Pravilno izolujte sve priključke.
- Redovno provjeravajte ispravnost anemometra i podizača roletni.
- Ne otvarajte uređaj pod naponom.

---

## Autor

- [Eldar6776](https://github.com/eldar6776)

---

## Licence

Uređaj koristi STM32 HAL biblioteku. Softver je dostupan "AS-IS", bez garancije.

---

Za sva pitanja i podršku, otvorite issue na [repozitoriju](https://github.com/eldar6776/WindSwitch).
