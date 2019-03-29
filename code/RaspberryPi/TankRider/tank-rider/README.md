# UselessTank TankRider
Remote control tank

## Setup Guide
### Environment Setup
1. Install node.js.
    * Set proxy for npm after install node.js, run following command in bash:
        ```bash
        npm config set proxy http://proxy.pal.sap.corp:8080/
        npm config set https-proxy http://proxy.pal.sap.corp:8080/
        ```

2. Install Angular CLI
    * To install the CLI using npm, run the following command:
      ```bash
        npm install -g @angular/cli
        ```
### Project Setup
1. Clone the project & setup.
    * Run following command in bash:
        ```bash
        git clone https://github.wdf.sap.corp/cicn/UselessTank.git 
        cd ‎⁨UselessTank⁩/code⁩/RaspberryPi⁩/TankRider⁩/tank-rider⁩
        npm install
        ```
        
2. Start the service
    * Run following command in bash:
        ```bash
        ng serve
        ```
    * Access page in your browser:
        * URL: http://localhost:4200/
       