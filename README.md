Witajcie chlopaki !
Dzialamy tak, ze kazdy bedzie mial zrobiona swa galaz, tzw. "branch".
Kazdy branch bedzie odpowiadal za jedna czesc projektu, tj: bluetooth, silniki
oraz mpu.
Pierwsze co to musicie sobie skopiowac aktualna wersje repozytorium do siebie lokalnie.
Jest to zawsze pierwsza czynnosc ktora wykonujemy zaczynajac prace.
Robimy to komenda "git clone URL" (za pierwszym razem). Potem juz wystarcza "git pull".
Nastepnie "checkoutujemy" sie na swojego branch'a komenda "git checkout BRANCHNAME".
Pracujemy, piszemy, rozwalamy mozgi i nastepnie "git add ." doda wszystkie pliki
do tzw. STAGED AREA. Teraz wystarczy owe pliki tylko "commit'owac", czyli stworzyc
krotkie podsumowanie tego co stworzylismy komenda "git commit -m "WIADOMOSC INFO".
Pozostaje juz tylko wypchnac to na serwer komenda "git push origin NAZWA_BRANCZA ".

Na koncu gdy praca bedzie skonczona (lub wczesniej gdy ktos bedzie pewny tego co zrobil i bedzie chcial to zachowac, zrobimy polaczenie brancha z masterem komenda "git merge".

git merge origin/serverfix -> JESLI NIE MASZ WLASNEJ KOPII REPO KOLEGI (niesprawdzone:) )
LUB
git checkout -b serverfix origin/serverfix -> dizeki temu od razu checkoutujesz sie na to repo (Sprawdzone)

Komenda "git status" sprawdza aktualny stan plikow.
Komenda "git log" pokazuje logi.


I najwazniejsze: potezny manual mamy tutaj: 
http://git-scm.com/book/en/v2

# -FreeScale-Selfbalancing
Source codes for self-balancing robot
