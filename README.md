# Deck Driver para SGP40

Este repositório contém o deck driver para o sensor de compostos orgânicos voláteis (VOC) **SGP40**. Este deck driver foi desenvolvido para a aeronave **Crazyflie 2.1** da Bitcraze.

---

## Descrição

O sensor **SGP40** é utilizado para medir compostos orgânicos voláteis, permitindo monitorar a qualidade do ar interno.  

<img width="165" height="220" alt="Sensor" src="https://github.com/user-attachments/assets/d7e45ac0-79fe-4539-9a97-7cf503aa53ff" />

Este projeto implementa um deck driver para integrar o sensor SGP40 à plataforma **Crazyflie 2.1**, utilizando comunicação I²C.

O desenvolvimento do deck seguiu as instruções da Bitcraze sobre como criar e implementar um deck driver ([Repositório Bitcraze - Deck Driver](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/howto/)) e levou em consideração as especificações contidas no datasheet do SGP40 ([Datasheet SGP40](https://sensirion.com/products/catalog/SGP40))

A medição do sensor é realizada a cada um segundo, seguindo o fluxo apresentado na figura abaixo.
O endereço I2C do sensor é hex.: 0x59.

<img width="603" height="245" alt="image" src="https://github.com/user-attachments/assets/7006ffdc-00fd-489a-8dd9-235c77c163f7" />






