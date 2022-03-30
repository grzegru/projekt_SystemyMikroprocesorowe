# projekt_SystemyMikroprocesorowe

Głównym zamysłem projektu było wykonanie układu automatycznej regulacji. W naszym przypadku zdecydowaliśmy się na sterowanie temperaturą poprzez podgrzewanie ogniwo Peltiera. Zdecydowaliśmy na wariant z użyciem
ogniwa Peltiera, a nie rezystora grzewczego ponieważ w przypadku rezystora bardzo długo musieliśmy czekać na
efekt. Ogniwo peltiera wraz z czujnikiem temperatury został przyklejony do radiatora w celu lepszej transmisji
ciepła co przekłada się na sprawniejsze działanie.
Temperatura jest zadawana poprzez obrót enkoderem. Jeden krok enkodera powoduje zmianę temperatury zadanej o 0.5◦C. 
Temperatura zadana jak i aktualna temperatura są wyświetlane na wyświetlaczu.
Projekt został wykonany przy użyciu płytki mikroprocesorowej Nucleo 746zg. Program został napisany w środowisku STM32CubeID.
Ogniwo peltiera jest zasilany zewnętrznym zasilaczem o napięciu 5V.



![image](https://user-images.githubusercontent.com/65861697/160862150-7b363a86-18ab-4d2a-b4e2-9eac581ebc89.png) 
