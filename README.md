# AlertWay Guardian - Women and Child Safety Device

#### How to Setup the Environment Variables for the Microphone WebSocket Server

1. Create a `.env` file in the root directory of the project:

```bash
touch .env
```

2. Add the following configurations to your `.env` file:

```plaintext
# Azure Storage Configuration
AZURE_STORAGE_CONNECTION_STRING=your_azure_storage_connection_string
AZURE_CONTAINER_NAME=your_container_name

# Server Configuration
SERVER_HOST=0.0.0.0
SERVER_PORT=8080
```

##### Environment Variables Reference

| Variable                          | Description                             | Required | Default   |
| --------------------------------- | --------------------------------------- | -------- | --------- |
| `AZURE_STORAGE_CONNECTION_STRING` | Azure Storage account connection string | Yes      | None      |
| `AZURE_CONTAINER_NAME`            | Name of the Azure Storage container     | Yes      | None      |
| `SERVER_HOST`                     | Host address for the WebSocket server   | No       | '0.0.0.0' |
| `SERVER_PORT`                     | Port number for the WebSocket server    | No       | 8080      |
