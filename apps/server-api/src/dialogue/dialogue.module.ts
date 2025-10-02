import { Module } from '@nestjs/common'
import { DialogueService } from './dialogue.service'

@Module({
  providers: [DialogueService],
  exports: [DialogueService],
})
export class DialogueModule {}
