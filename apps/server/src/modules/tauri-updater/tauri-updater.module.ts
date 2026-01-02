import { Module } from '@nestjs/common'
import { TauriUpdaterController } from './tauri-updater.controller'
import { TauriUpdaterService } from './tauri-updater.service'

@Module({
  controllers: [TauriUpdaterController],
  providers: [TauriUpdaterService],
})
export class TauriUpdaterModule {}
