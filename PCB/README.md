For our groups PCB we decided to build a custom RP2040 microcontroller which was responsible for handling inputs from the encoders and sending data out to the motor drivers.  The main reason we decided to design a custom RP2040 microcontroller instead of simply using a Raspberry Pi Pico, had to do with us wanting to use widley available connectors to attach and remove motors and encoders from the board.

The Raspberry Pi Foundation publishes an abundant amount of information on the Raspberry Pi Pico microcontroller, including the full schematics and board layout files.  However, these design files are only available in Cadence Allegro, which UMass does not have a licence for - and I do not have the money for.  As a result I was limited to PDF sources, and had to undergo the process of reverse-engineering a Raspberry Pi Pico H in KiCad 8.0 utilizing datasheets and other miscelanious PDFs provided by the Raspberry Pi Foundation. 

Below is a 3D render of my board.  This version is much better than the original prototype we had created for our CDR presentation.  Compared to our original prototype, our second and final revision has more efficiently placed items, incorperates components that are more abundantly available, has rounded edges, and an overall more visually pleasing appearance.  I have included the project files which include schematic drawings, PCB layouts, and gerber export files.

<p align="center">
  <img width="651" alt="Screenshot 2024-05-22 at 1 19 52 PM" src="https://github.com/Samalmeida1028/sdp-team-12/assets/78173414/bcb54056-28de-4d3b-9093-767ba9433222">
</p>
